from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional

from .sensor import Sensor
from .tactile_harness import TactileHarness
from .tactile_specs import get_tactile_spec


class TactileProfileBase(Sensor):
    WORLD_T1 = "tactile_t1_threshold.world"
    WORLD_T2 = "tactile_t2_uniformity.world"
    WORLD_T3 = "tactile_t3_stability.world"
    WORLD_T4 = "tactile_t4_peak_load.world"

    def __init__(self, CONFIG):
        super().__init__()
        root = Path(CONFIG.get("ROOT_PATH", "") or "")
        sensors_root = Path(CONFIG["SENSORS_PATH"])
        if not sensors_root.is_absolute():
            sensors_root = root / sensors_root

        self.sensor_sdf_path = str((sensors_root / self.sensor_type / self.sensor_name / "model.sdf").resolve())
        self.result_root = root / "results" / self.sensor_name
        self.metrics_dir = self.result_root / "metrics"
        self.metrics_dir.mkdir(parents=True, exist_ok=True)
        self.spec = get_tactile_spec(self.sensor_name)
        self._params: Dict[str, Any] = {}
        self.test_to_world = {
            "t1_minimum_force_test": self.WORLD_T1,
            "t2_uniformity_test": self.WORLD_T2,
            "t3_stability_test": self.WORLD_T3,
            "t4_peak_load_test": self.WORLD_T4,
        }

    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        duration_s: float = 0.5,
        **_: Any,
    ) -> Optional[Dict[str, Any]]:
        harness = TactileHarness(self.sensor_name, simulator=simulator, result_root=self.result_root)
        if world_path:
            harness.open_world(Path(world_path).name)
        stream = harness.make_stream()
        try:
            samples = harness.collect_window(stream, duration_s=float(duration_s))
            return {
                "samples": harness.serialize_samples(samples),
                "summary": harness.sample_summary(samples),
            }
        finally:
            stream.stop()

    def set_params(self, **params) -> None:
        self._params.update(params)

    def get_params(self) -> Dict[str, Any]:
        return {
            "sensor_name": self.sensor_name,
            "sensor_type": self.sensor_type,
            "sensor_sdf_path": self.sensor_sdf_path,
            "rated_force_n": self.spec.rated_force_n,
            "rated_torque_nm": self.spec.rated_torque_nm,
            "active_zone_m": self.spec.active_zone_m,
            **self._params,
        }

    @staticmethod
    def _default_assumptions() -> List[str]:
        return [
            "Normal loading is applied along the world Z axis onto a horizontally mounted sensor.",
            "The tactile response metric is the absolute Z component of the published wrench force.",
            TactileHarness.SENSOR_PLACEMENT_WARNING,
        ]

    def t1_minimum_force_test(self, simulator) -> Dict[str, Any]:
        harness = TactileHarness(self.sensor_name, simulator=simulator, result_root=self.result_root)
        harness.open_world(self.WORLD_T1)
        stream = harness.make_stream()
        try:
            steps = []
            threshold_force = None
            for force_n in [0.5, 1.0, 1.5, 2.0, 2.5]:
                harness.clear_body_wrenches()
                harness.apply_body_wrench((0.0, 0.0, -force_n), duration_s=0.6)
                samples = harness.collect_window(stream, duration_s=0.7)
                summary = harness.sample_summary(samples)
                stable_slice = samples[-max(20, len(samples) // 4):] if samples else []
                stable_summary = harness.sample_summary(stable_slice or samples)
                steps.append(
                    {
                        "command_force_n": float(force_n),
                        "response_summary": summary,
                        "stable_response_summary": stable_summary,
                        "samples": harness.serialize_samples(samples),
                    }
                )
                if (
                    stable_summary["mean_normal_force"] >= (0.8 * force_n)
                    and stable_summary["std_normal_force"] <= 0.20 * max(force_n, 1e-6)
                ):
                    threshold_force = float(force_n)
                    break

            payload = {
                "steps": steps,
                "responses": [step["response_summary"]["mean_normal_force"] for step in steps],
                "threshold_force": threshold_force,
                "pass_criterion_n": 2.0,
                "assumptions": self._default_assumptions() + [
                    "Stable triggering is evaluated on the last quarter of the sampling window to suppress the initial application transient.",
                    "Stable triggering is treated as mean response >= 80% of commanded load with <= 20% tail-window standard deviation.",
                ],
            }
            payload["metric_path"] = harness.write_metric("t1_minimum_force_test", payload)
            if threshold_force is None or threshold_force > 2.0:
                raise AssertionError(f"T1 failed: threshold_force={threshold_force}")
            return payload
        finally:
            stream.stop()

    def t2_uniformity_test(self, simulator) -> Dict[str, Any]:
        harness = TactileHarness(self.sensor_name, simulator=simulator, result_root=self.result_root)
        harness.open_world(self.WORLD_T2)
        stream = harness.make_stream()
        half_zone = self.spec.active_zone_m / 2.0
        offsets = [-half_zone, 0.0, half_zone]
        grid = []
        try:
            for y in offsets:
                row = []
                for x in offsets:
                    harness.clear_body_wrenches()
                    harness.apply_body_wrench((0.0, 0.0, -1.96), duration_s=0.7, reference_point_xyz=(x, y, 0.0))
                    samples = harness.collect_window(stream, duration_s=0.8)
                    summary = harness.sample_summary(samples)
                    row.append(
                        {
                            "x_m": float(x),
                            "y_m": float(y),
                            "response_n": float(summary["mean_normal_force"]),
                            "samples": harness.serialize_samples(samples),
                        }
                    )
                grid.append(row)

            flat = [cell["response_n"] for row in grid for cell in row]
            mean_response = sum(flat) / len(flat) if flat else 0.0
            max_relative_deviation = max(
                (abs(value - mean_response) / mean_response) if mean_response else 0.0
                for value in flat
            ) if flat else 0.0
            payload = {
                "response_grid_3x3": grid,
                "mean_response": float(mean_response),
                "max_relative_deviation": float(max_relative_deviation),
                "command_force_n": 1.96,
                "active_zone_m": float(self.spec.active_zone_m),
                "assumptions": self._default_assumptions() + [
                    "The 0.05 x 0.05 m active zone is centered on the sensor origin.",
                    "The 3 x 3 grid samples are taken at -0.025, 0.0 and +0.025 m offsets along X/Y.",
                ],
            }
            payload["metric_path"] = harness.write_metric("t2_uniformity_test", payload)
            if max_relative_deviation > 0.10:
                raise AssertionError(f"T2 failed: max_relative_deviation={max_relative_deviation}")
            return payload
        finally:
            stream.stop()

    def t3_stability_test(self, simulator) -> Dict[str, Any]:
        duration_s = float(self._params.get("stability_duration_s", 600.0))
        sample_window_s = float(self._params.get("stability_sample_window_s", 1.0))
        harness = TactileHarness(self.sensor_name, simulator=simulator, result_root=self.result_root)
        harness.open_world(self.WORLD_T3)
        stream = harness.make_stream()
        try:
            harness.clear_body_wrenches()
            harness.apply_body_wrench((0.0, 0.0, -4.9), duration_s=duration_s)
            series = []
            elapsed = 0.0
            while elapsed < duration_s:
                samples = harness.collect_window(stream, duration_s=sample_window_s)
                summary = harness.sample_summary(samples)
                series.append(
                    {
                        "elapsed_s": float(elapsed),
                        "mean_normal_force": float(summary["mean_normal_force"]),
                        "std_normal_force": float(summary["std_normal_force"]),
                        "peak_normal_force": float(summary["peak_normal_force"]),
                    }
                )
                elapsed += sample_window_s

            harness.clear_body_wrenches()
            recovery_samples = harness.collect_window(stream, duration_s=5.0)
            recovery_summary = harness.sample_summary(recovery_samples)
            initial_value = float(series[0]["mean_normal_force"]) if series else 0.0
            final_value = float(series[-1]["mean_normal_force"]) if series else 0.0
            relative_change = abs(final_value - initial_value) / initial_value if initial_value else math.inf
            drift_n = max(abs(point["mean_normal_force"] - initial_value) for point in series) if series else 0.0
            chatter_n = max(point["std_normal_force"] for point in series) if series else 0.0

            payload = {
                "time_series": series,
                "initial_value": initial_value,
                "final_value": final_value,
                "relative_change": float(relative_change),
                "recovery_value": float(recovery_summary["mean_normal_force"]),
                "drift_n": float(drift_n),
                "chatter_n": float(chatter_n),
                "command_force_n": 4.9,
                "duration_s": float(duration_s),
                "assumptions": self._default_assumptions() + [
                    "Recovery is sampled during a 5 second unloaded observation window after the hold phase.",
                    "Drift is reported as the largest absolute deviation from the initial mean normal-force estimate.",
                    "Chatter is reported as the largest 1 second windowed standard deviation during the hold phase.",
                ],
            }
            payload["metric_path"] = harness.write_metric("t3_stability_test", payload)
            if relative_change > 0.05:
                raise AssertionError(f"T3 failed: relative_change={relative_change}")
            return payload
        finally:
            stream.stop()

    def t4_peak_load_test(self, simulator) -> Dict[str, Any]:
        capture_duration_s = float(self._params.get("peak_capture_duration_s", 3.0))
        harness = TactileHarness(self.sensor_name, simulator=simulator, result_root=self.result_root)
        harness.open_world(self.WORLD_T4)
        stream = harness.make_stream()
        try:
            waveform_samples = harness.collect_window(stream, duration_s=capture_duration_s, poll_timeout_s=0.05)
            peak_value = max((sample.normal_force for sample in waveform_samples), default=0.0)
            plateau_hits = sum(
                1 for sample in waveform_samples
                if abs(sample.normal_force - peak_value) <= max(0.02 * peak_value, 1e-6)
            )
            saturation_detected = peak_value >= 0.98 * self.spec.rated_force_n or plateau_hits >= 5
            payload = {
                "waveform": harness.serialize_samples(waveform_samples),
                "peak_value": float(peak_value),
                "saturation_detected": bool(saturation_detected),
                "capture_duration_s": float(capture_duration_s),
                "rated_force_n": float(self.spec.rated_force_n),
                "assumptions": self._default_assumptions() + [
                    "The impact body is the cylinder defined in tactile_t4_peak_load.world: diameter 0.05 m, height 0.1 m, mass 1 kg.",
                    "Saturation/clipping is inferred when the peak reaches >= 98% of the rated force or forms a repeated plateau at the peak.",
                ],
            }
            payload["metric_path"] = harness.write_metric("t4_peak_load_test", payload)
            if saturation_detected:
                raise AssertionError(f"T4 failed: saturation_detected={saturation_detected}, peak_value={peak_value}")
            return payload
        finally:
            stream.stop()
