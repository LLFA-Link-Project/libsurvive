#include <cnkalman/kalman.h>
#include <stdio.h>
#include "cnkalman_internal.h"

static inline FLT mul_at_b_a(const struct CnMat *A, const struct CnMat *B) {
    CN_CREATE_STACK_MAT(V, 1, 1);
    assert(A->cols == 1);
    if (B->cols > 1) {
		CN_CREATE_STACK_MAT(AtiB, 1, B->rows);
        cnGEMM(A, B, 1, 0, 0, &AtiB, CN_GEMM_FLAG_A_T);
        cnGEMM(&AtiB, A, 1, 0, 0, &V, 0);
    } else {
		CN_CREATE_STACK_VEC(AtiB, B->rows);
        cnElementwiseMultiply(&AtiB, A, B);
        V.data[0] = cnDot(&AtiB, A);
    }

    return V.data[0];
}

CN_EXPORT_FUNCTION FLT calculate_v_meas(const struct CnMat *y, const struct CnMat *iR) {
        return .5 * mul_at_b_a(y, iR);
}
CN_EXPORT_FUNCTION FLT calculate_v_delta(const struct CnMat *xDiff, const struct CnMat *iP) {
    return .5 * mul_at_b_a(xDiff, iP);
}
CN_EXPORT_FUNCTION FLT calculate_v(const struct CnMat *y, const struct CnMat *xDiff, const struct CnMat *iR,
                                   const struct CnMat *iP, FLT *meas_part, FLT *delta_part) {
    if (delta_part == 0) {
        return *meas_part = calculate_v_meas(y, iR);
    }
    *meas_part = calculate_v_meas(y, iR);
    *delta_part = calculate_v_delta(xDiff, iP);
    return  *meas_part + *delta_part;
}

const char *cnkalman_update_extended_termination_reason_to_str(
        enum cnkalman_update_extended_termination_reason reason) {
    switch (reason) {
        case cnkalman_update_extended_termination_reason_none:
            return "none";
        case cnkalman_update_extended_termination_reason_maxiter:
            return "maxiter";
        case cnkalman_update_extended_termination_reason_invalid_jacobian:
            return "invalid_jac";
        case cnkalman_update_extended_termination_reason_xtol:
            return "xtol";
        case cnkalman_update_extended_termination_reason_MAX:
            return "MAX";
        case cnkalman_update_extended_termination_reason_step:
            return "step";
        case cnkalman_update_extended_termination_reason_mtol:
            return "mtol";
		case cnkalman_update_extended_termination_reason_too_high_error:
			return "maxerr";
        default:
            return "";
    }
}

static inline enum cnkalman_update_extended_termination_reason
cnkalman_termination_criteria(const struct term_criteria_t *term_criteria,
                                    FLT initial_error, FLT error, FLT alpha, FLT last_error) {
    FLT minimum_step = term_criteria->minimum_step > 0 ? term_criteria->minimum_step : .01;
    if (alpha == 0 || alpha < minimum_step) {
        return cnkalman_update_extended_termination_reason_step;
    }
    if (error == 0) {
        return cnkalman_update_extended_termination_reason_xtol;
    }

    if (isfinite(last_error) && fabs(last_error - error) < term_criteria->xtol * error) {
        return cnkalman_update_extended_termination_reason_xtol;
    }
    return cnkalman_update_extended_termination_reason_none;
}

static void cnkalman_find_error_state(void* user, cnkalman_state_t *k, const CnMat* prior, const CnMat* point, CnMat* error_state) {
	if (k->ErrorState_fn) {
		k->ErrorState_fn(user, prior, point, error_state, 0);
	} else {
		cnSub(error_state, point, prior);
	}
}

// Linearize the original prior in the current iteration's error coordinates.
// e = boxminus(point, prior), J = d e / d delta at point boxplus delta.
// Its local covariance is J^-1 P J^-T and its local mean step is -J^-1 e.
// Keep the original precision for evaluating the actual nonlinear objective.
static FLT cnkalman_prior_at_iteration(void *user, cnkalman_state_t *k, const CnMat *prior,
                                     const CnMat *point, const CnMat *prior_P, const CnMat *precision,
                                     CnMat *mean_step, CnMat *negative_gradient) {
    int n = k->error_state_size;
    CN_CREATE_STACK_VEC(e, n);
    CN_CREATE_STACK_VEC(weighted_error, n);
    CN_CREATE_STACK_MAT(J, n, n);
    CN_CREATE_STACK_MAT(inverse_J, n, n);
    if (k->ErrorState_fn) {
        CN_CREATE_STACK_MAT(de_dx, n, k->state_cnt);
        CN_CREATE_STACK_MAT(dx_ddelta, k->state_cnt, n);
        k->ErrorState_fn(user, prior, point, &e, &de_dx);
        if (k->Update_fn) k->Update_fn(user, point, NULL, NULL, &dx_ddelta);
        else cn_eye(&dx_ddelta, 0);
        cnGEMM(&de_dx, &dx_ddelta, 1, NULL, 0, &J, 0);
        cnInvert(&J, &inverse_J, CN_INVERT_METHOD_LU);
    } else {
        cnSub(&e, point, prior);
        cn_eye(&J, 0);
        cn_eye(&inverse_J, 0);
    }
    if (!cn_is_finite(&e) || !cn_is_finite(&inverse_J)) return INFINITY;
    CN_CREATE_STACK_MAT(product, n, n);
    cnGEMM(&inverse_J, prior_P, 1, NULL, 0, &product, 0);
    cnGEMM(&product, &inverse_J, 1, NULL, 0, &k->P, CN_GEMM_FLAG_B_T);
    cnGEMM(&inverse_J, &e, -1, NULL, 0, mean_step, 0);
    cnGEMM(precision, &e, 1, NULL, 0, &weighted_error, 0);
    cnGEMM(&J, &weighted_error, -1, NULL, 0, negative_gradient, CN_GEMM_FLAG_A_T);
    return calculate_v_delta(&e, precision);
}

void cnkalman_update_state(void* user, cnkalman_state_t *k, const CnMat* x0, FLT scale, const CnMat* error_state, CnMat* x1) {
	if(k->Update_fn) {
		CN_CREATE_STACK_MAT(error_state_delta_scaled, k->error_state_size, 1);
		cnScale(&error_state_delta_scaled, error_state, scale);
		k->Update_fn(user, x0, &error_state_delta_scaled, x1, 0);
	} else {
		cnAddScaled(x1, error_state, scale, x0, 1);
	}
}

#define FOR_EACH_STATE(mk) \
for(int i = 0, filter_idx = 0, state_idx = 0;i < mk->ks_cnt;i++) {\
cnkalman_state_t * k = mk->ks[i];                                 \
CnMat error_state_delta_view = cnMatView(k->error_state_size, 1, &error_state_delta, filter_idx, 0);\
CnMat xn_view = cnMatView(k->state_cnt, 1, &xn, state_idx, 0);    \
CnMat iPdx_view = cnMatView(k->error_state_size, 1, &iPdx, filter_idx, 0);\
CnMat x_i_view = cnMatConstView(k->state_cnt, 1, &x_i, state_idx, 0);                         \
CnMat error_state_view = cnMatView(k->error_state_size, 1, &error_state, filter_idx, 0);\
const CnMat x_k_k1_view = cnMatConstView(k->state_cnt, 1, x_k_k1, state_idx, 0);\


#define END_FOR_EACH_STATE()\
filter_idx += k->error_state_size;\
state_idx += k->state_cnt;\
}

// Extended Kalman Filter Modifications Based on an Optimization View Point
// https://www.diva-portal.org/smash/get/diva2:844060/FULLTEXT01.pdf
// Note that in this document, 'y' is the measurement; which we refer to as 'Z'
// throughout this code and so 'y - h(x)' from the paper is 'y' in code.
// The main driver in this document is V(x) which is defined as:
// r(x) = [ R^-.5 * y; P^-.5 * (x_t-1 * x) ]
// V(X) = r'(x) * r(x) / 2
// V(X) = 1/2 * (R^-.5 * y)' * R^-.5 * y + (P^-.5 * (x_t-1 * x))'*P^-.5 * (x_t-1 * x)
// Then owing to the identity (AB)' = B'A', and also to the fact that R and P are symmetric and so R' = R; P' = P:
// V(X) = 1/2 * (y' * (R^-.5)' * R^-.5 * y + (x_t-1 * x)' * (P^-.5)'* P^-.5 * (x_t-1 * x))
// V(X) = 1/2 * (y' * (R^-.5) * R^-.5 * y + (x_t-1 * x)' * (P^-.5)* P^-.5 * (x_t-1 * x))
// V(X) = 1/2 * (y' * (R^-1) * y + (x_t-1 * x)' * (P^-1) * (x_t-1 * x))

// Similarly, we need dV(X)/dX -- ΔV(X) --
// ΔV(X) = -[ R^-.5 * H; P^-.5]' * r(x)
// ΔV(X) = -[ R^-.5 * H; P^-.5]' * [ R^-.5 * y; P^-.5 * (x_t-1 * x) ]
// ΔV(X) =  -(R^-.5 * H) * (R^-.5 * y) - P^-.5 * (P^-.5 * (x_t-1 * x))
// ΔV(X) =  H' * R^-1 * y - P^-1 * (x_t-1 * x)
// The point of all of this is that we don't need to ever explicitly calculate R^-.5 / P^-.5; just the inverses
FLT cnkalman_run_iterations(cnkalman_meas_model_t *mk, const struct CnMat *Z, const struct CnMat *R,
                                         void *user, const CnMat *x_k_k1, CnMat *K,
                                         CnMat *H, CnMat *x_k_k, struct cnkalman_update_extended_stats_t *stats) {
    int state_cnt = cnkalman_model_state_count(mk);
    int filter_cnt = cnkalman_model_filter_count(mk);

    int meas_cnt = Z->rows;

    CN_CREATE_STACK_MAT(y, meas_cnt, 1);
    CN_CREATE_STACK_MAT(x_i, state_cnt, 1);
    CN_CREATE_STACK_MAT(x_i_best, state_cnt, 1);

    CN_CREATE_STACK_MAT(iR, meas_cnt, R->cols > 1 ? meas_cnt : 1);
    if (R->cols > 1) {
        cnInvert(R, &iR, CN_INVERT_METHOD_LU);
    } else {
        for (int i = 0; i < meas_cnt; i++) {
            cn_as_vector(&iR)[i] = cn_as_const_vector(R)[i] == 0 ? 0 : 1. / cn_as_const_vector(R)[i];
        }
    }
    // kalman_print_mat_v(k, 100, "iR", &iR, true);

    CnMat iPS[CNKALMAN_STATES_PER_MODEL] = { 0 };
    CnMat priorPS[CNKALMAN_STATES_PER_MODEL] = { 0 };
    for(int i = 0;i < mk->ks_cnt;i++) {
        CN_CREATE_STACK_MAT(iP, mk->ks[i]->error_state_size, mk->ks[i]->error_state_size);
        CN_CREATE_STACK_MAT(prior_P, mk->ks[i]->error_state_size, mk->ks[i]->error_state_size);
        cnCopy(&mk->ks[i]->P, &prior_P, 0);
        priorPS[i] = prior_P;
        cnInvert(&mk->ks[i]->P, &iP, CN_INVERT_METHOD_SVD);
        iPS[i] = iP;

        assert(sane_covariance(&mk->ks[i]->P));
        assert(cn_is_finite(&iP));
    }

    assert(cn_is_finite(&iR));

    enum cnkalman_update_extended_termination_reason stop_reason =
            cnkalman_update_extended_termination_reason_none;
    FLT current_norm = INFINITY, last_norm = INFINITY;
    FLT initial_norm = 0;

    cn_matrix_copy(&x_i, x_k_k1);
    int iter;
    int max_iter = mk->term_criteria.max_iterations;
    FLT meas_part = 0, delta_part = 0;
    CN_CREATE_STACK_MAT(Hxdiff, Z->rows, 1);
	CN_CREATE_STACK_MAT(error_state_delta, filter_cnt, 1);
	CN_CREATE_STACK_MAT(error_state_delta_scaled, filter_cnt, 1);
    CN_CREATE_STACK_MAT(xn, x_i.rows, x_i.cols);
    CN_CREATE_STACK_MAT(error_state, filter_cnt, 1);
    CN_CREATE_STACK_MAT(iRy, meas_cnt, 1);
    CN_CREATE_STACK_MAT(iPdx, filter_cnt, 1);
    CN_CREATE_STACK_MAT(dVt, filter_cnt, 1);

	CN_CREATE_STACK_VEC(verify_x, state_cnt);
	cn_set_zero(&error_state);
	cn_set_zero(&iPdx);

	// `x_k_k1` is prior state
	// `xi` is current iteration state; initializes as `x_k_k1`
	// `error_state` is the difference in xi and x_k_k1
	// `error_state_delta` is the max gain change for this iteration step; which is scaled down if its
	//        too expensive
    for (iter = 0; iter < max_iter && stop_reason == cnkalman_update_extended_termination_reason_none; iter++) {
        // Find the residual y and possibly also the jacobian H. The user could have passed one in as 'user', or given
        // us a map function which will calculate it.
        struct CnMat *HR = cnkalman_find_residual(mk, user, Z, &x_i, &y, H);
        if (stats) {
            stats->fevals++;
            stats->hevals++;
        }

        // If the measurement jacobian isn't calculable, the best we can do is just bail.
        if (HR == 0) {
            stop_reason = cnkalman_update_extended_termination_reason_invalid_jacobian;
            current_norm = INFINITY;
            break;
        }
        if (HR != H) cnCopy(HR, H, 0);
        last_norm = current_norm;

        // Re-express the prior around this iterate before computing its gain.
        delta_part = 0;
		if(iter != 0) {
		    FOR_EACH_STATE(mk)
                delta_part += cnkalman_prior_at_iteration(user, k, &x_k_k1_view, &x_i_view,
                    &priorPS[i], &iPS[i], &error_state_view, &iPdx_view);
		    END_FOR_EACH_STATE()
		}
        meas_part = calculate_v_meas(&y, &iR);

        current_norm = meas_part + delta_part;
        if (!isfinite(current_norm)) {
            stop_reason = cnkalman_update_extended_termination_reason_invalid_jacobian;
            break;
        }
        assert(current_norm >= 0);

		if (iter == 0) {
			initial_norm = current_norm;
			if (stats) {
				stats->orignorm_meas += meas_part;
				stats->origerror = cnNorm(&y);
			}

			if(initial_norm > mk->term_criteria.max_error && mk->term_criteria.max_error > 0) {
				stop_reason = cnkalman_update_extended_termination_reason_too_high_error;
				break;
			}
		}
		//printf("%s iteration %2d: %7.7f/%7.7f %7.7f %7.7f\n", mk->name, iter, current_norm, initial_norm, meas_part, delta_part);

        if (R->cols > 1) {
            cnGEMM(&iR, &y, 1, 0, 0, &iRy, 0);
        } else {
            cnElementwiseMultiply(&iRy, &iR, &y);
        }

		// dVt = H * (R^-1 * y)' - iP * error_state
		cnGEMM(H, &iRy, -1, &iPdx, -1, &dVt, CN_GEMM_FLAG_A_T);

        // Run update; filling in K
        cnkalman_find_k(mk, K, H, R);

        if ((stop_reason =
                     cnkalman_termination_criteria(&mk->term_criteria, initial_norm, current_norm, 1, last_norm)) >
            cnkalman_update_extended_termination_reason_none) {
            goto end_of_loop;
        }

        // error_state_delta = K * (z - y - H * Δx)
		if(iter != 0) {
			cnGEMM(H, &error_state, 1, 0, 0, &Hxdiff, 0);
			cnSub(&y, &y, &Hxdiff);
		}
        cnGEMM(K, &y, 1, &error_state, 1, &error_state_delta, 0);

        FLT scale = 1.;
        FLT m = cnDot(&dVt, &error_state_delta);
        if (fabs(m) < mk->term_criteria.mtol) {
            stop_reason = cnkalman_update_extended_termination_reason_mtol;
            break;
        }
        FLT c = .5, tau = .5;
        FLT fa = 0, fa_best = current_norm, a_best = 0;

        FLT min_step = mk->term_criteria.minimum_step == 0 ? .05 : mk->term_criteria.minimum_step;

        bool exit_condition = false;
        while (!exit_condition) {
            exit_condition = true;

			// xn = x_i + scale * error_state
            FOR_EACH_STATE(mk)
                cnkalman_update_state(user, k, &x_i_view, scale, &error_state_delta_view, &xn_view);
            END_FOR_EACH_STATE()

			if(mk->Hfn) {
				mk->Hfn(user, Z, &xn, &y, 0);
			} else {
				cnGEMM((struct CnMat *)user, &xn, -1, Z, 1, &y, 0);
			}

            if (stats) {
                stats->fevals++;
            }

            fa = 0;
			// error_state = x_k_k1 - xn
			// xn = x_i_view * scale * error_state_delta
            FOR_EACH_STATE(mk)
                cnkalman_find_error_state(user, k, &x_k_k1_view, &xn_view, &error_state_view);
                cnkalman_update_state(user, k, &x_i_view, scale, &error_state_delta_view, &xn_view);
                fa += calculate_v_delta(&error_state_view, &iPS[i]);
            END_FOR_EACH_STATE()
            meas_part = calculate_v_meas(&y, &iR);
			//printf("\tscale  %f: %7.7f/%7.7f %7.7f %7.7f\n", scale, current_norm, initial_norm, meas_part, fa);
			fa += meas_part;

			if (fa_best > fa) {
				fa_best = fa;
				a_best = scale;
			}

            if (fa >= current_norm + scale * m * c) {
                exit_condition = false;

                scale = tau * scale;

                if (scale <= min_step) {
                    current_norm = fa_best;
                    scale = a_best;
                    break;
                }
            }
        }

        if (stats && stats->total_stats) {
            stats->total_stats->step_cnt++;
            stats->total_stats->step_acc += scale;
        }

		// x_i = x_i + scale * error_state_delta
		//printf("\tupdate %f: %7.7f/%7.7f\n", scale, fa_best, initial_norm);
        FOR_EACH_STATE(mk)
            cnkalman_update_state(user, k, &x_i_view, scale, &error_state_delta_view, &x_i_view);

            if (k->normalize_fn) {
                k->normalize_fn(k->user, &x_i_view);
            }
            assert(cn_is_finite(&x_i));
        END_FOR_EACH_STATE()

        end_of_loop:
        if (stop_reason == cnkalman_update_extended_termination_reason_none)
            stop_reason =
                    cnkalman_termination_criteria(&mk->term_criteria, initial_norm, current_norm, scale, last_norm);
    }
    if (stop_reason == cnkalman_update_extended_termination_reason_none)
        stop_reason = cnkalman_update_extended_termination_reason_maxiter;

    // The final accepted step may end the loop without another linearization.
    // Both gain and covariance must describe that accepted nominal state.
    CnMat *final_H = NULL;
    if (stop_reason != cnkalman_update_extended_termination_reason_invalid_jacobian &&
        stop_reason != cnkalman_update_extended_termination_reason_too_high_error)
        final_H = cnkalman_find_residual(mk, user, Z, &x_i, &y, H);
    if (final_H) {
        if (final_H != H) cnCopy(final_H, H, 0);
        delta_part = 0;
        FOR_EACH_STATE(mk)
            delta_part += cnkalman_prior_at_iteration(user, k, &x_k_k1_view, &x_i_view,
                &priorPS[i], &iPS[i], &error_state_view, &iPdx_view);
        END_FOR_EACH_STATE()
        meas_part = calculate_v_meas(&y, &iR);
        current_norm = meas_part + delta_part;
        if (isfinite(current_norm)) cnkalman_find_k(mk, K, H, R);
        if (stats) { stats->fevals++; stats->hevals++; }
    } else {
        current_norm = INFINITY;
        if (stop_reason != cnkalman_update_extended_termination_reason_too_high_error)
            stop_reason = cnkalman_update_extended_termination_reason_invalid_jacobian;
    }
    bool isFailure = current_norm > initial_norm || !isfinite(current_norm);
    if (stats) {
        stats->besterror = cnNorm(&y);
        stats->iterations = iter;
        stats->orignorm = initial_norm;
        stats->bestnorm = current_norm;
        stats->stop_reason = stop_reason;
        stats->bestnorm_meas = meas_part;
        stats->bestnorm_delta = delta_part;
        if (stats->total_stats) {
            stats->total_stats->total_runs++;
            stats->total_stats->total_failures += isFailure;
            stats->total_stats->orignorm_acc += initial_norm;
            stats->total_stats->bestnorm_acc += current_norm;
            stats->total_stats->stop_reason_counts[stop_reason]++;
            stats->total_stats->total_fevals += stats->fevals;
            stats->total_stats->total_hevals += stats->hevals;
            stats->total_stats->total_iterations += stats->iterations;
            stats->total_stats->bestnorm_meas_acc += stats->bestnorm_meas;
            stats->total_stats->bestnorm_delta_acc += stats->bestnorm_delta;
            stats->total_stats->orignorm_meas_acc += stats->orignorm_meas;
        }
    }

    if (isFailure) {
        initial_norm = -1;
        for (int i = 0; i < mk->ks_cnt; i++) cnCopy(&priorPS[i], &mk->ks[i]->P, 0);
    } else {
        assert(cn_is_finite(H));
        assert(cn_is_finite(K));
        cn_matrix_copy(x_k_k, &x_i);
    }

    CN_FREE_STACK_MAT(Hxdiff);
    CN_FREE_STACK_MAT(error_state_delta);
    CN_FREE_STACK_MAT(xn);
    CN_FREE_STACK_MAT(x_diff);
    CN_FREE_STACK_MAT(iRy);
    CN_FREE_STACK_MAT(iPdx);
    CN_FREE_STACK_MAT(dVt);
    CN_FREE_STACK_MAT(y);
    CN_FREE_STACK_MAT(x_i);
    CN_FREE_STACK_MAT(x_i_best);

    CN_FREE_STACK_MAT(iR);
    CN_FREE_STACK_MAT(iP);

    return initial_norm;
}
