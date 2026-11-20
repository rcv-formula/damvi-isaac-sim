"""Independent steering model audit; never changes runtime configuration.

Reuses extracted signals, then evaluates models with an embargoed nested temporal
split and recording-held-out split. Run with the analysis environment documented
by extract_steering_bags.py. All angles are radians internally.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import maximum_filter1d, minimum_filter1d
from scipy.optimize import least_squares

from fit_steering_nonlinear import (
    KNOTS, LIMIT, collect, nonlinear_fit, predict as pchip_predict,
)

MODELS = ("affine_clamped", "asymmetric_clamped", "monotone_cubic_clamped", "monotone_pchip",
          "servo_clamped_affine", "servo_clamped_asymmetric", "servo_clamped_pchip")
PENALTIES = (0., .00003, .0003, .003)
BINS = np.array([-.9, -.6, -.3, 0., .3, .6, .9])
DEG = 180 / np.pi
SERVO_COMMAND_LIMIT = .35 / .52
SERVO_KNOTS = np.array([-SERVO_COMMAND_LIMIT, -.5, -.3, -.15, 0., .15, .3, .5, SERVO_COMMAND_LIMIT])


def servo_pchip_fit(x, y, penalty):
    """Known VESC servo clipping is enforced before fitting the interior map."""
    from scipy.interpolate import PchipInterpolator
    initial = .4175 * SERVO_KNOTS + .0145
    gaps = np.diff(np.r_[-LIMIT, initial, LIMIT])
    initial_parameters = np.log(gaps[:-1] / gaps[-1])

    def angles(parameters):
        logs = np.r_[parameters, 0.]
        gaps = np.exp(logs - np.max(logs))
        gaps /= gaps.sum()
        return -LIMIT + 2 * LIMIT * np.cumsum(gaps[:-1])

    def residuals(parameters):
        values = angles(parameters)
        pred = PchipInterpolator(SERVO_KNOTS, values)(np.clip(x, -SERVO_COMMAND_LIMIT, SERVO_COMMAND_LIMIT))
        residual = (pred - y) / .02
        data = np.sign(residual) * np.sqrt(2 * .02**2 * (np.sqrt(1 + residual**2) - 1) / len(x))
        slope = np.diff(values) / np.diff(SERVO_KNOTS)
        return np.r_[data, np.sqrt(penalty / (len(slope) - 1)) * np.diff(slope)]

    result = least_squares(residuals, initial_parameters, bounds=(-20, 20),
                           ftol=1e-9, xtol=1e-9, gtol=1e-9, max_nfev=1500)
    if not result.success:
        raise RuntimeError(result.message)
    return np.maximum.accumulate(angles(result.x))


def fit_model(name, x, y, penalty=.0003):
    if name == "monotone_pchip":
        return nonlinear_fit(x, y, penalty).tolist()
    if name == "servo_clamped_pchip":
        return servo_pchip_fit(x, y, penalty).tolist()
    if name in ("affine_clamped", "servo_clamped_affine"):
        initial, bounds = [.01, .4], ([-.08, .05], [.08, 1.2])
    elif name in ("asymmetric_clamped", "servo_clamped_asymmetric"):
        initial, bounds = [.01, .4, .4], ([-.08, .05, .05], [.08, 1.2, 1.2])
    elif name == "monotone_cubic_clamped":
        # f'(u) = gain * (1 + curvature * (u / input_limit)**2) > 0.
        initial, bounds = [.01, .4, 0.], ([-.08, .05, -.95], [.08, 1.2, 3.])
    else:
        raise ValueError(name)
    return least_squares(
        lambda p: evaluate(name, p, x) - y, initial, bounds=bounds,
        loss="soft_l1", f_scale=.02, ftol=1e-10, xtol=1e-10, gtol=1e-10,
    ).x.tolist()


def evaluate(name, p, x):
    p = np.asarray(p)
    if name == "monotone_pchip":
        return pchip_predict(p, x)
    if name.startswith("servo_clamped"):
        x = np.clip(x, -SERVO_COMMAND_LIMIT, SERVO_COMMAND_LIMIT)
        if name == "servo_clamped_pchip":
            from scipy.interpolate import PchipInterpolator
            return PchipInterpolator(SERVO_KNOTS, p)(x)
        name = {"servo_clamped_affine": "affine_clamped",
                "servo_clamped_asymmetric": "asymmetric_clamped"}[name]
    if name == "affine_clamped":
        z = p[0] + p[1] * x
    elif name == "asymmetric_clamped":
        z = p[0] + p[1] * np.maximum(x, 0) + p[2] * np.minimum(x, 0)
    elif name == "monotone_cubic_clamped":
        z = p[0] + p[1] * (x + p[2] * x**3 / (3 * KNOTS[-1]**2))
    else:
        raise ValueError(name)
    return np.clip(z, -LIMIT, LIMIT)


def rmse(error):
    return float(DEG * np.sqrt(np.mean(np.asarray(error)**2)))


def split(t, available, fold, block_s, folds, embargo=.5):
    group = np.floor(t / block_s).astype(int) % folds
    test = available & (group == fold)
    # Purge neighboring training rows so their centered smoothing support cannot
    # overlap a test block. The maximum preprocessing half-window is 0.25 s.
    before = np.floor((t - embargo) / block_s).astype(int) % folds
    after = np.floor((t + embargo) / block_s).astype(int) % folds
    train = available & (group != fold) & (before != fold) & (after != fold)
    return train, test


def choose_penalty(x, y, t, available, model="monotone_pchip"):
    losses = []
    for penalty in PENALTIES:
        squared = []
        for fold in range(3):
            train, test = split(t, available, fold, 10., 3)
            if train.sum() < 80 or test.sum() < 10:
                continue
            p = fit_model(model, x[train], y[train], penalty)
            squared.extend((evaluate(model, p, x[test]) - y[test])**2)
        losses.append({"penalty": penalty, "inner_rmse_deg": float(DEG * np.sqrt(np.mean(squared)))})
    best = min(losses, key=lambda q: q["inner_rmse_deg"])
    # Predeclared 0.03 degree tolerance; use the smoothest near-tied candidate.
    chosen = max((q for q in losses if q["inner_rmse_deg"] <= best["inner_rmse_deg"] + .03),
                 key=lambda q: q["penalty"])
    return chosen["penalty"], losses


def score_predictions(y, pred, bag, blocks, x, datasets):
    error = pred - y
    finite = np.isfinite(error)
    per_bag = []
    for bi, (name, _) in enumerate(datasets):
        m = finite & (bag == bi)
        if m.any():
            per_bag.append({"bag": name, "samples": int(m.sum()), "rmse_deg": rmse(error[m]),
                            "bias_deg": float(DEG * error[m].mean())})
    per_bin = []
    for low, high in zip(BINS[:-1], BINS[1:]):
        m = finite & (x >= low) & (x < high)
        if m.any():
            per_bin.append({"input_low_rad": float(low), "input_high_rad": float(high),
                            "samples": int(m.sum()), "blocks_2s": len(np.unique(blocks[m])),
                            "rmse_deg": rmse(error[m])})
    return {
        "rmse_deg": rmse(error[finite]),
        "mae_deg": float(DEG * np.abs(error[finite]).mean()),
        "p95_absolute_error_deg": float(DEG * np.quantile(np.abs(error[finite]), .95)),
        "bag_macro_rmse_deg": float(np.mean([q["rmse_deg"] for q in per_bag])),
        "input_bin_macro_rmse_deg": float(np.mean([q["rmse_deg"] for q in per_bin])),
        "per_bag": per_bag, "per_input_bin": per_bin,
    }


def cross_validate(x, y, bag, blocks, t, datasets):
    predictions = {name: np.full_like(y, np.nan) for name in MODELS}
    folds = []
    for fold in range(5):
        train, test = split(t, np.ones(len(t), bool), fold, 20., 5)
        penalty, inner = choose_penalty(x, y, t, train)
        servo_penalty, servo_inner = choose_penalty(x, y, t, train, "servo_clamped_pchip")
        item = {"fold": fold, "train_samples": int(train.sum()), "test_samples": int(test.sum()),
                "pchip_penalty": penalty, "pchip_inner_candidates": inner,
                "servo_pchip_penalty": servo_penalty, "servo_pchip_inner_candidates": servo_inner, "models": {}}
        for name in MODELS:
            p = fit_model(name, x[train], y[train], servo_penalty if name == "servo_clamped_pchip" else penalty)
            pred = evaluate(name, p, x[test])
            predictions[name][test] = pred
            item["models"][name] = {"parameters": p, "rmse_deg": rmse(pred - y[test])}
        folds.append(item)
        print("Temporal fold", fold, {n: round(q["rmse_deg"], 3) for n, q in item["models"].items()}, flush=True)
    scores = {n: score_predictions(y, p, bag, blocks, x, datasets) for n, p in predictions.items()}
    return {"block_size_s": 20., "folds": 5, "embargo_s": .5,
            "inner_block_size_s": 10., "inner_folds": 3, "scores": scores,
            "fold_details": folds}, predictions


def leave_bag_out(x, y, bag, blocks, t, datasets):
    predictions = {name: np.full_like(y, np.nan) for name in MODELS}
    folds = []
    for bi in np.unique(bag):
        train, test = bag != bi, bag == bi
        penalty, inner = choose_penalty(x, y, t, train)
        servo_penalty, servo_inner = choose_penalty(x, y, t, train, "servo_clamped_pchip")
        item = {"held_bag": datasets[bi][0], "train_samples": int(train.sum()),
                "test_samples": int(test.sum()), "pchip_penalty": penalty,
                "pchip_inner_candidates": inner, "servo_pchip_penalty": servo_penalty,
                "servo_pchip_inner_candidates": servo_inner, "models": {}}
        for name in MODELS:
            p = fit_model(name, x[train], y[train], servo_penalty if name == "servo_clamped_pchip" else penalty)
            pred = evaluate(name, p, x[test])
            predictions[name][test] = pred
            item["models"][name] = {"parameters": p, "rmse_deg": rmse(pred - y[test])}
        folds.append(item)
        print("Held recording", datasets[bi][0], {n: round(q["rmse_deg"], 3) for n, q in item["models"].items()}, flush=True)
    scores = {n: score_predictions(y, p, bag, blocks, x, datasets) for n, p in predictions.items()}
    return {"scores": scores, "fold_details": folds}, predictions


def paired_bootstrap(y, predictions, bag, t):
    # Paired resampling of 20-second temporal clusters, rather than 50 Hz rows.
    clusters = bag * 10000 + np.floor(t / 20.).astype(int)
    unique, inverse = np.unique(clusters, return_inverse=True)
    count = np.bincount(inverse)
    sums = {n: np.bincount(inverse, weights=(p - y)**2) for n, p in predictions.items()}
    rng = np.random.default_rng(20261113)
    draws = rng.integers(0, len(unique), (2000, len(unique)))
    base = DEG * np.sqrt(sums["affine_clamped"][draws].sum(axis=1) / count[draws].sum(axis=1))
    result = {}
    for name in MODELS[1:]:
        candidate = DEG * np.sqrt(sums[name][draws].sum(axis=1) / count[draws].sum(axis=1))
        result[name] = {"rmse_difference_to_affine_deg_95pct": np.quantile(candidate - base, [.025, .975]).tolist(),
                        "fraction_bootstraps_better_than_affine": float(np.mean(candidate < base))}
    return {"cluster_size_s": 20., "clusters": len(unique), "resamples": 2000, "comparisons": result}


def auxiliary_signals(datasets, bag, t):
    keys = ("vpf", "a", "gyro", "r")
    out = {k: np.empty(len(t)) for k in keys}
    out["steering_span_200ms"] = np.empty(len(t))
    for bi, (_, d) in enumerate(datasets):
        m = bag == bi
        if not m.any():
            continue
        for k in keys:
            out[k][m] = np.interp(t[m], d["t"], d[k])
        command = np.nan_to_num(d["u"])
        span = maximum_filter1d(command, 11) - minimum_filter1d(command, 11)
        out["steering_span_200ms"][m] = np.interp(t[m] - .07, d["t"], span)
    return out


def sensitivity(x, y, bag, v, auxiliary):
    masks = {
        "primary": np.ones(len(x), bool),
        "pf_speed_match_10pct": np.abs(auxiliary["vpf"] / v - 1) <= .1,
        "pf_speed_match_5pct": np.abs(auxiliary["vpf"] / v - 1) <= .05,
        "minimum_speed_1mps": v >= 1.,
        "lateral_accel_max_0_7mps2": np.abs(auxiliary["r"] * v) <= .7,
        "command_span_200ms_below_0_05rad": auxiliary["steering_span_200ms"] < .05,
    }
    result = []
    for name, m in masks.items():
        if m.sum() < 50:
            continue
        params = fit_model("monotone_pchip", x[m], y[m], .0003)
        affine = fit_model("affine_clamped", x[m], y[m])
        result.append({"filter": name, "samples": int(m.sum()), "seconds": float(m.sum() * .02),
                       "input_range_rad": [float(x[m].min()), float(x[m].max())],
                       "affine_parameters": affine, "pchip_angles_rad": params,
                       "sample_counts_per_bag_index": {str(i): int(np.sum(m & (bag == i))) for i in np.unique(bag)}})
    # A separate localized speed is a sensitivity check, not ground truth.
    y_pf = np.arctan(.26 * auxiliary["r"] / auxiliary["vpf"])
    params = fit_model("monotone_pchip", x, y_pf, .0003)
    result.append({"filter": "pf_position_speed_instead_of_wheel_speed", "samples": len(x),
                   "pchip_angles_rad": params, "is_ground_truth": False})
    return result


def nuisance_bag_offsets(x, y, bag):
    other_bags = np.unique(bag)[1:]
    offsets = [(bag == i).astype(float) for i in other_bags]
    result = {}
    for cubic in (False, True):
        matrix = np.column_stack([np.ones(len(x)), x] + ([x**3] if cubic else []) + offsets)
        p = least_squares(lambda q: matrix @ q - y, np.linalg.lstsq(matrix, y, rcond=None)[0],
                          loss="soft_l1", f_scale=.02).x
        result["cubic_with_bag_offsets" if cubic else "affine_with_bag_offsets"] = {
            "parameters": p.tolist(), "rmse_deg_training_only": rmse(matrix @ p - y),
            "reference_bag_index": int(np.unique(bag)[0]), "offset_bag_indices": other_bags.tolist(),
            "purpose": "diagnose recording-dependent trim/slip; these offsets are not deployable without recording identity",
        }
    return result


def gap_audit(cache, datasets, bag, t):
    contaminated = np.zeros(len(t), bool)
    details = []
    for bi, (name, _) in enumerate(datasets):
        with np.load(cache / (name + ".npz")) as d:
            row = {"bag": name, "channels": {}}
            m = bag == bi
            for key, col in (("imu", 1), ("core", 0), ("pf", 0)):
                times = np.sort(d[key][:, col])
                gaps = np.flatnonzero(np.diff(times) > .1)
                near = np.zeros(m.sum(), bool)
                for i in gaps:
                    near |= (t[m] >= times[i] - .3) & (t[m] <= times[i + 1] + .3)
                contaminated[m] |= near
                row["channels"][key] = {"gaps_above_100ms": len(gaps), "selected_rows_near_gap": int(near.sum())}
            details.append(row)
    return {"excluded_support_halfwidth_s": .3, "selected_rows_near_gap": int(contaminated.sum()),
            "selected_rows_near_gap_fraction": float(contaminated.mean()), "details": details}, contaminated


def servo_echo_audit(cache, output, bag_root):
    """Verify limits using the driver's post-clipping echo, not its input topic."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    result = []
    for path in sorted(bag_root.rglob("*.mcap")):
        name = path.parent.name + "_" + path.stem
        cached = output / (path.parent.name + "_servo_telemetry.npz")
        if cached.exists():
            telemetry = np.load(cached)["telemetry"]
        else:
            with path.open("rb") as stream:
                reader = make_reader(stream, decoder_factories=[DecoderFactory()])
                base = reader.get_summary().statistics.message_start_time // 1_000_000_000
                telemetry = np.array([[message.log_time * 1e-9 - base, decoded.data]
                                      for _, _, message, decoded in reader.iter_decoded_messages(
                                          topics=["/sensors/servo_position_command"])])
            np.savez_compressed(cached, telemetry=telemetry)
        raw = np.load(cache / (name + ".npz"))
        command = raw["servo"]
        command = command[np.argsort(command[:, 0], kind="stable")]
        i = np.clip(np.searchsorted(command[:, 0], telemetry[:, 0], side="right") - 1, 0, len(command) - 1)
        age = telemetry[:, 0] - command[i, 0]
        matched = (age >= 0) & (age < .01)
        difference = telemetry[:, 1] - np.clip(command[i, 1], .15, .85)
        result.append({
            "bag": name, "source_mcap": str(path), "telemetry_count": len(telemetry),
            "receipt_matched_within_10ms": int(matched.sum()),
            "observed_min_max_normalized": [float(telemetry[:, 1].min()), float(telemetry[:, 1].max())],
            "fraction_equal_to_clipped_command_within_1e_6": float(np.mean(np.abs(difference[matched]) < 1e-6)),
            "abs_difference_to_clipped_command_quantiles": np.quantile(np.abs(difference[matched]), [.5, .95, .99, 1.]).tolist(),
        })
    return {"servo_input_topic": "/commands/servo/position",
            "servo_post_clipping_echo_topic": "/sensors/servo_position_command",
            "meaning": "The echo is the commanded PWM position after driver clipping, not a measured shaft/wheel angle.",
            "normalized_limits": [.15, .85], "nominal_gain_observed": .52, "nominal_offset_observed": .5,
            "nominal_limit_rad": SERVO_COMMAND_LIMIT,
            "driver_source": "/home/shin/Desktop/DAMVI/src/damvi_ws/f1tenth_system/vesc/vesc_driver/src/vesc_driver.cpp",
            "checked_out_parameter_sign_note": "The checked-out vesc.yaml currently says -0.52; these recorded commands agree with +0.52. Recorded behavior is authoritative for this fit.",
            "recordings": result}


def recommended_mapping(x, y, bag, t, report, output):
    """Use the observed saturation with the least-complex near-tied interior."""
    import yaml
    p = report["full_data_parameters_diagnostic_only"]["servo_clamped_affine"]
    ends = evaluate("servo_clamped_affine", p, np.array([-SERVO_COMMAND_LIMIT, SERVO_COMMAND_LIMIT]))
    clusters = bag * 10000 + np.floor(t / 20).astype(int)
    unique = np.unique(clusters)
    members = {c: np.flatnonzero(clusters == c) for c in unique}
    rng = np.random.default_rng(20261113)
    bootstrap = []
    for _ in range(300):
        rows = np.concatenate([members[c] for c in rng.choice(unique, len(unique), replace=True)])
        bootstrap.append(fit_model("servo_clamped_affine", x[rows], y[rows]))
    bootstrap = np.asarray(bootstrap)
    config = yaml.safe_load((output.parent / "steering" / "steering_nonlinear.yaml").read_text())
    config["input_limit_rad"] = SERVO_COMMAND_LIMIT
    config["input_knots_rad"] = [-SERVO_COMMAND_LIMIT, SERVO_COMMAND_LIMIT]
    config["wheel_angle_knots_rad"] = ends.tolist()
    config["calibration"].pop("nonlinear_regularization", None)
    config["calibration"].update({
        "method": "Observed driver servo clipping, robust affine interior in IMU kinematic equivalent angle",
        "report": "../calibration/steering_review/report.json",
        "blocked_holdout_rmse_deg": report["nested_temporal_cv"]["scores"]["servo_clamped_affine"]["rmse_deg"],
        "validation_protocol": "Nested temporal cross-validation: 20 s outer blocks, 5 folds, 0.5 s embargo",
        "leave_recording_out_rmse_deg": report["leave_recording_out"]["scores"]["servo_clamped_affine"]["rmse_deg"],
        "servo_command_gain_observed": .52, "servo_command_offset_observed": .5,
        "servo_normalized_min": .15, "servo_normalized_max": .85,
        "mechanical_wheel_angle_measured": False,
        "mapping_status": "Provisional equivalent angle; linkage curvature inside saturation is not identified",
    })
    (output / "recommended_steering.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    # Optional curved interior is supplied for inspection, without recommending
    # extra curvature that did not improve validation.
    curved = dict(config)
    curved["input_knots_rad"] = SERVO_KNOTS.tolist()
    curved["wheel_angle_knots_rad"] = report["full_data_parameters_diagnostic_only"]["servo_clamped_pchip"]
    curved["calibration"] = dict(config["calibration"])
    curved["calibration"]["mapping_status"] = "Alternative only; additional interior curvature did not improve nested CV"
    curved["calibration"]["nonlinear_regularization"] = .0003
    curved["calibration"]["blocked_holdout_rmse_deg"] = report["nested_temporal_cv"]["scores"]["servo_clamped_pchip"]["rmse_deg"]
    curved["calibration"]["leave_recording_out_rmse_deg"] = report["leave_recording_out"]["scores"]["servo_clamped_pchip"]["rmse_deg"]
    (output / "alternative_curved_steering.yaml").write_text(yaml.safe_dump(curved, sort_keys=False))
    return {
        "model_family": "known servo input clamp, affine interior",
        "runtime_representation": "monotone_pchip with 2 knots; this represents the affine interior exactly and clamps outside",
        "input_saturation_rad": [-SERVO_COMMAND_LIMIT, SERVO_COMMAND_LIMIT],
        "offset_rad": p[0], "gain": p[1], "input_knots_rad": config["input_knots_rad"],
        "wheel_angle_knots_rad": config["wheel_angle_knots_rad"],
        "bootstrap_20s_clusters": len(unique), "bootstrap_samples": len(bootstrap),
        "offset_gain_temporal_bootstrap_95pct": np.quantile(bootstrap, [.025, .975], axis=0).tolist(),
        "reason": "Observed software saturation is mandatory. Interior candidate errors are practically tied; use the least-complex curve until measured mechanical wheel-angle data identify linkage curvature.",
        "global_mapping_is_nonlinear": True,
        "interior_linkage_nonlinearity_established": False,
    }


def plot_clipping(cache, output, audit, recommended):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4), constrained_layout=True)
    rng = np.random.default_rng(1538)
    for item in audit["recordings"]:
        stem = item["bag"]
        prefix = Path(item["source_mcap"]).parent.name
        telemetry = np.load(output / (prefix + "_servo_telemetry.npz"))["telemetry"]
        command = np.load(cache / (stem + ".npz"))["servo"]
        command = command[np.argsort(command[:, 0], kind="stable")]
        i = np.clip(np.searchsorted(command[:, 0], telemetry[:, 0], side="right") - 1, 0, len(command) - 1)
        age = telemetry[:, 0] - command[i, 0]
        selected = np.flatnonzero((age >= 0) & (age < .01))
        selected = rng.choice(selected, min(1500, len(selected)), replace=False)
        axes[0].scatter((command[i[selected], 1] - .5) / .52 * DEG,
                        (telemetry[selected, 1] - .5) / .52 * DEG,
                        s=5, alpha=.25, label=stem)
    grid = np.linspace(-.9, .9, 1000)
    axes[0].plot(grid * DEG, np.clip(grid, -SERVO_COMMAND_LIMIT, SERVO_COMMAND_LIMIT) * DEG, "k--", lw=1.5)
    axes[0].set(xlabel="Nominal steering input (deg)", ylabel="Post-clipping command equivalent (deg)",
                title="Recorded VESC driver command saturation")
    axes[0].legend(fontsize=6)
    angle = evaluate("servo_clamped_affine", [recommended["offset_rad"], recommended["gain"]], grid)
    axes[1].plot(grid * DEG, angle * DEG, lw=2, color="#305e8b")
    axes[1].scatter(np.array(recommended["input_knots_rad"]) * DEG,
                    np.array(recommended["wheel_angle_knots_rad"]) * DEG, c="#305e8b", zorder=3)
    axes[1].set(xlabel="Nominal steering input (deg)", ylabel="Provisional equivalent wheel angle (deg)",
                title="Recommended map: observed clamp, minimal interior")
    for ax in axes:
        ax.axvline(-SERVO_COMMAND_LIMIT * DEG, color="gray", alpha=.4)
        ax.axvline(SERVO_COMMAND_LIMIT * DEG, color="gray", alpha=.4)
        ax.grid(alpha=.2)
    fig.savefig(output / "servo_clipping.png", dpi=180)
    plt.close(fig)


def plot_review(output, report, datasets, x, y, bag, fitted):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    grid = np.linspace(KNOTS[0], KNOTS[-1], 500)
    for bi, (name, _) in enumerate(datasets):
        m = bag == bi
        if m.any():
            axes[0, 0].scatter(x[m] * DEG, y[m] * DEG, s=5, alpha=.2, label=name)
    axes[0, 0].plot(grid * DEG, evaluate("affine_clamped", fitted["affine_clamped"], grid) * DEG, "k--", label="Clamped affine")
    axes[0, 0].plot(grid * DEG, evaluate("monotone_pchip", fitted["monotone_pchip"], grid) * DEG, "k", lw=2, label="Current nonlinear family")
    axes[0, 0].plot(grid * DEG, evaluate("servo_clamped_affine", fitted["servo_clamped_affine"], grid) * DEG,
                    color="#3971a6", lw=1.5, label="Observed servo clip + affine interior")
    axes[0, 0].set(xlabel="Nominal command (deg)", ylabel="IMU equivalent angle (deg)", title="Equivalent angle; no direct wheel-angle measurement")
    axes[0, 0].legend(fontsize=6)
    plot_models = ("affine_clamped", "monotone_pchip", "servo_clamped_affine", "servo_clamped_pchip")
    for j, name in enumerate(plot_models):
        temporal = report["nested_temporal_cv"]["scores"][name]["rmse_deg"]
        held = report["leave_recording_out"]["scores"][name]["rmse_deg"]
        axes[0, 1].bar(j - .18, temporal, width=.36, color="#3971a6", label="Nested temporal CV" if j == 0 else None)
        axes[0, 1].bar(j + .18, held, width=.36, color="#c88a35", label="Recording held out" if j == 0 else None)
    axes[0, 1].set_xticks(range(len(plot_models)), ["Affine +\nangle limit", "Free PCHIP", "Known servo\nclip + affine", "Known servo\nclip + PCHIP"])
    axes[0, 1].set(ylabel="Out-of-fold RMSE (deg)", title="Complexity must improve unseen data")
    axes[0, 1].legend(fontsize=8)
    for bi, (name, _) in enumerate(datasets):
        m = bag == bi
        if m.sum() >= 80:
            angles = fit_model("monotone_pchip", x[m], y[m], .003)
            lo, hi = np.quantile(x[m], [.01, .99])
            valid = (grid >= lo) & (grid <= hi)
            axes[1, 0].plot(grid[valid] * DEG, evaluate("monotone_pchip", angles, grid[valid]) * DEG, label=name)
    axes[1, 0].set(xlabel="Nominal command (deg)", ylabel="Fitted equivalent angle (deg)", title="Recording-to-recording response varies")
    axes[1, 0].legend(fontsize=7)
    for item in report["filter_sensitivity"]:
        if item["filter"] in ("primary", "pf_speed_match_5pct", "minimum_speed_1mps", "command_span_200ms_below_0_05rad"):
            lo, hi = item["input_range_rad"]
            valid = (grid >= lo) & (grid <= hi)
            axes[1, 1].plot(grid[valid] * DEG, evaluate("monotone_pchip", item["pchip_angles_rad"], grid[valid]) * DEG,
                            label=f'{item["filter"]} (n={item["samples"]})')
    axes[1, 1].set(xlabel="Nominal command (deg)", ylabel="Fitted equivalent angle (deg)", title="Filter sensitivity; endpoint shape is uncertain")
    axes[1, 1].legend(fontsize=6)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.savefig(output / "review.png", dpi=180)
    plt.close(fig)


def run(aligned, output, bag_root):
    datasets, samples = collect(aligned, .07)
    x, y, bag, blocks, t, speed = samples.T
    bag, blocks = bag.astype(int), blocks.astype(int)
    output.mkdir(parents=True, exist_ok=True)
    temporal, temporal_pred = cross_validate(x, y, bag, blocks, t, datasets)
    recordings, recording_pred = leave_bag_out(x, y, bag, blocks, t, datasets)
    aux = auxiliary_signals(datasets, bag, t)
    filters = sensitivity(x, y, bag, speed, aux)
    gap_details, near_gap = gap_audit(aligned.parent / "cache", datasets, bag, t)
    if near_gap.any():
        m = ~near_gap
        gap_details["pchip_without_near_gap_rows_rad"] = fit_model("monotone_pchip", x[m], y[m], .0003)
    support = []
    for low, high in zip(np.linspace(-.9, .9, 13)[:-1], np.linspace(-.9, .9, 13)[1:]):
        m = (x >= low) & (x < high)
        support.append({"input_low_rad": float(low), "input_high_rad": float(high), "samples": int(m.sum()),
                        "blocks_2s": len(np.unique(blocks[m])),
                        "per_bag": {datasets[i][0]: int(np.sum(m & (bag == i))) for i in np.unique(bag) if np.sum(m & (bag == i))}})
    fitted = {name: fit_model(name, x, y, .0003) for name in MODELS}
    lag_sensitivity = []
    for lag in (0., .04, .07, .1, .15, .2, .3):
        _, ss = collect(aligned, lag)
        xx, yy = ss[:, 0], ss[:, 1]
        fit = fit_model("affine_clamped", xx, yy)
        lag_sensitivity.append({"lag_s": lag, "samples": len(xx), "affine_parameters": fit,
                                "training_rmse_deg": rmse(evaluate("affine_clamped", fit, xx) - yy)})
    report = {
        "sample_count": len(x), "selected_seconds": float(len(x) * .02),
        "source_aligned_directory": str(aligned.resolve()),
        "fixed_lag_s": .07, "fixed_lag_note": "Inherited from prior data exploration, not a new blind latency estimate. All compared models share it.",
        "preprocessing_support_s": {"yaw_window": .3, "speed_window": .3, "pf_position_window": .5},
        "selection": {"speed_min_mps": .6, "speed_max_mps": 2.5, "longitudinal_accel_max_mps2": .5,
                      "lateral_accel_max_mps2": 1.5, "command_servo_tolerance_rad": .04},
        "datasets": [name for name, _ in datasets], "nested_temporal_cv": temporal,
        "leave_recording_out": recordings,
        "temporal_paired_block_bootstrap": paired_bootstrap(y, temporal_pred, bag, t),
        "filter_sensitivity": filters, "input_support": support,
        "recording_offset_diagnostic": nuisance_bag_offsets(x, y, bag),
        "acquisition_gap_audit": gap_details, "lag_sensitivity": lag_sensitivity,
        "full_data_parameters_diagnostic_only": fitted,
        "pchip_input_knots_rad": KNOTS.tolist(),
        "servo_clipped_pchip_input_knots_rad": SERVO_KNOTS.tolist(),
        "observed_servo_limits_normalized": [.15, .85],
        "known_nominal_input_saturation_rad": [-SERVO_COMMAND_LIMIT, SERVO_COMMAND_LIMIT],
        "servo_echo_audit": servo_echo_audit(aligned.parent / "cache", output, bag_root),
        "mechanical_angle_identifiable": False,
        "limitations": [
            "The label atan(0.26*yaw_rate/wheel_speed) is a kinematic equivalent angle; slip, effective tire radius, yaw dynamics and trim cannot be separated from linkage geometry.",
            "No cmd_vel or actual front-wheel-angle topic exists in these five recordings; /drive is matched to the executed normalized servo command as a proxy.",
            "The original single holdout was also used to choose PCHIP regularization. This review uses nested selection and an independent held-recording protocol to reduce that optimism.",
            "Temporal cross-validation remains a same-recording estimate. Held-recording error measures transfer to a different run and is more conservative.",
            "The mechanical angle bound is constrained by the INI; the exact linkage curvature inside the observed software saturation limits is not determined.",
            "The known servo command clamp is a separately observed nonlinearity. It does not establish curvature of the mechanical linkage inside that range.",
        ],
    }
    report["recommendation"] = recommended_mapping(x, y, bag, t, report, output)
    (output / "report.json").write_text(json.dumps(report, indent=2))
    (output / "servo_telemetry_audit.json").write_text(json.dumps(report["servo_echo_audit"], indent=2))
    with (output / "out_of_fold_predictions.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "time_s", "command_rad", "equivalent_angle_rad"] + ["temporal_" + n for n in MODELS] + ["held_recording_" + n for n in MODELS])
        w.writerows([datasets[int(bag[i])][0], t[i], x[i], y[i]] + [temporal_pred[n][i] for n in MODELS] + [recording_pred[n][i] for n in MODELS] for i in range(len(x)))
    plot_review(output, report, datasets, x, y, bag, fitted)
    plot_clipping(aligned.parent / "cache", output, report["servo_echo_audit"], report["recommendation"])
    print(json.dumps({"temporal": {n: q["rmse_deg"] for n, q in temporal["scores"].items()},
                      "held_recording": {n: q["rmse_deg"] for n, q in recordings["scores"].items()},
                      "bootstrap": report["temporal_paired_block_bootstrap"]}, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("aligned", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--bag-root", type=Path, default=Path("/home/shin/Desktop/red-damvi dataset"))
    args = parser.parse_args()
    run(args.aligned, args.output, args.bag_root)
