"""Explicit bounded calibration profiles, selected before any device is opened."""

from dataclasses import asdict, dataclass, replace
import math

from .plan import Limits
from .solver import CalibrationLimits
from .stability import StabilityLimits


@dataclass(frozen=True)
class CalibrationProfile:
    name: str
    motion: Limits
    fit: CalibrationLimits
    stability: StabilityLimits

    def as_dict(self):
        return asdict(self)

    def describe(self):
        precision = ('约 5°粗标定，局部 95% 角度不确定度估计上限 5°，不是绝对精度保证'
                     if self.name == 'wide' else '保守档，保持原质量容限')
        return '\n'.join([
            f'标定档位 {self.name}：{self.motion.inner_tilt_deg:g}°/{self.motion.outer_tilt_deg:g}°；{precision}。',
            f'采集质量上限：预检趋势 {self.stability.force_slope_limit_n_per_s*60:g} N/min 且首尾变化 '
            f'{self.stability.endpoint_force_delta_limit_n:g} N 超限时拒绝；回中力差 '
            f'{self.stability.reference_force_limit_n:g} N、矩差 {self.stability.reference_torque_limit_nm:g} Nm。',
            f'最终拟合 RMSE ≤{self.fit.max_fit_rmse_n:g} N、独立姿态验证 RMSE ≤{self.fit.max_validation_rmse_n:g} N；'
            '仍检查相对误差和作用方符号可辨识性，不扣除漂移。',
        ])


def get_profile(name='standard'):
    if name == 'standard':
        return CalibrationProfile(name, Limits(), CalibrationLimits(), StabilityLimits())
    if name == 'wide':
        return CalibrationProfile(name,
            replace(Limits(), inner_tilt_deg=20., outer_tilt_deg=45., interleave_tilts=True,
                    max_tilt_rad=math.radians(46.), joint_excursion_rad=1.5),
            replace(CalibrationLimits(), min_tilt_span_deg=70., min_gravity_axis_rms=.05,
                    max_fit_rmse_n=.20, max_fit_pose_error_n=.40, max_relative_fit_error=.20,
                    max_validation_rmse_n=.25, max_validation_pose_error_n=.50,
                    max_relative_validation_error=.25, max_rotation_uncertainty_deg=5.),
            replace(StabilityLimits(), reference_force_limit_n=.25,
                    force_slope_limit_n_per_s=.005, endpoint_force_delta_limit_n=.15))
    raise ValueError(f'unknown calibration profile: {name}')
