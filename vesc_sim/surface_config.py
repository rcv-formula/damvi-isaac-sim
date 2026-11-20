"""Resolve one constant ground-friction coefficient for a dataset at load time.

This does not alter tire construction parameters or evaluate position, speed or
time. A missing calibration explicitly falls back to the configured default.
"""

import hashlib
import math
from pathlib import Path

import yaml


def _coefficient(value, name, *, positive=False):
    if isinstance(value, bool) or value is None:
        raise ValueError(f'{name} must be a finite numeric coefficient')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name} must be a finite numeric coefficient') from error
    if not math.isfinite(result) or result < 0 or result > 3 or (positive and result == 0):
        bounds = '(0, 3]' if positive else '[0, 3]'
        raise ValueError(f'{name} must be finite and in {bounds}')
    return result


def load_surface_configuration(path, dataset=None):
    """Read and resolve the active dataset afresh, returning scalar provenance.

    ``unfitted_fallback`` means the selected profile exists but has no numeric
    coefficient. ``unknown_dataset_fallback_unfitted`` means it is absent.
    A numeric profile retains its supplied status, or ``configured`` if omitted.
    The original YAML bytes are hashed on every call, so Play-time reloads can
    report exactly which configuration was resolved.
    """
    source = Path(path).expanduser().resolve()
    raw = source.read_bytes()
    config = yaml.safe_load(raw)
    if not isinstance(config, dict):
        raise ValueError('Surface friction configuration must be a mapping')
    selected = config.get('selected_dataset') if dataset is None else dataset
    if not isinstance(selected, str) or not selected.strip():
        raise ValueError('selected_dataset must be a nonempty string')
    selected = selected.strip()
    reference = _coefficient(config.get('reference_coefficient'),
                             'reference_coefficient', positive=True)
    default = _coefficient(config.get('default_coefficient'), 'default_coefficient')
    profiles = config.get('datasets', {})
    if not isinstance(profiles, dict):
        raise ValueError('Surface datasets must be a mapping')

    if selected not in profiles:
        coefficient = default
        status = 'unknown_dataset_fallback_unfitted'
    else:
        profile = profiles[selected]
        if profile is None:
            profile = {}
        if not isinstance(profile, dict):
            raise ValueError('Selected surface dataset profile must be a mapping')
        if profile.get('coefficient') is None:
            coefficient = default
            status = 'unfitted_fallback'
        else:
            coefficient = _coefficient(profile['coefficient'],
                                       f'datasets[{selected!r}].coefficient')
            status = profile.get('status', 'configured')
            if not isinstance(status, str) or not status.strip():
                raise ValueError('Selected surface dataset status must be a nonempty string')
    return dict(selected_dataset=selected, coefficient=coefficient,
                reference_coefficient=reference, sourcepath=str(source),
                sha256=hashlib.sha256(raw).hexdigest(), status=status)
