#!/usr/bin/env python3
"""Check the rig's models, calibration files and camera serials before launch.

``run_rig.sh`` passes the exact (override-resolved) paths and serials its publisher
processes will use, so this validates what will really be loaded.  Exit 0 only when
every required item passes and no optional item that exists is corrupt.  numpy is imported lazily; pypylon/ultralytics are only
needed for the hardware checks.
"""

import argparse
import sys
import zipfile
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Result:
    label: str
    ok: bool
    required: bool
    detail: str
    skipped: bool = False


def check_model_file(label, path, required=True):
    """A PyTorch checkpoint is a zip archive containing a data.pkl member."""
    if not path:
        return Result(label, not required, required, "not configured", skipped=not required)
    path = Path(path)
    if not path.is_file():
        return Result(label, not required, required, f"missing: {path}", skipped=not required)
    if not zipfile.is_zipfile(path):
        return Result(label, False, required, f"not a PyTorch zip archive (incomplete copy?): {path}")
    try:
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
            bad = archive.testzip()
    except (zipfile.BadZipFile, OSError) as error:
        return Result(label, False, required, f"unreadable archive: {error}")
    if bad is not None:
        return Result(label, False, required, f"corrupt archive member {bad}")
    if not any(name.endswith("data.pkl") for name in names):
        return Result(label, False, required, "archive has no data.pkl; not a PyTorch checkpoint")
    return Result(label, True, required, f"{path} ({path.stat().st_size/1048576:.1f} MiB)")


def load_calibration(directory):
    """Return (camera_matrix, distortion) from a directory; raise ValueError if unusable."""
    directory = Path(directory)
    files = [directory / name for name in ("camera_matrix.npy", "distortion_coefficients.npy")]
    for file in files:
        if not file.is_file():
            raise ValueError(f"missing {file}")
    import numpy as np

    arrays = []
    for file in files:
        try:
            arrays.append(np.load(file, allow_pickle=False))
        except Exception as error:
            raise ValueError(f"cannot load {file}: {error}") from None
    matrix, distortion = (np.asarray(a, dtype=float) for a in arrays)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError(f"camera_matrix.npy must be a finite 3x3 array, got shape {matrix.shape}")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("camera_matrix.npy has non-positive focal lengths")
    distortion = distortion.reshape(-1)
    if not 4 <= distortion.size <= 14 or not np.isfinite(distortion).all():
        raise ValueError(
            f"distortion_coefficients.npy needs 4-14 finite values, got {distortion.size}")
    return matrix, distortion


def check_calibration(label, directory):
    if not directory:
        return Result(label, False, True, "not configured")
    try:
        load_calibration(directory)
    except ValueError as error:
        return Result(label, False, True, str(error))
    return Result(label, True, True, str(directory))


def check_serials(human, robot, attached):
    """Both serials set, distinct, and attached.  ``attached`` is a list of serial strings."""
    human, robot = (human or "").strip(), (robot or "").strip()
    results = []
    if not human or not robot:
        results.append(Result("camera serials", False, True,
                              "HUMAN_CAMERA_SERIAL and ROBOT_CAMERA_SERIAL must both be set"))
        return results
    if human == robot:
        results.append(Result("camera serials", False, True,
                              f"both publishers would open camera {human}; they need distinct cameras"))
        return results
    for label, serial in (("human camera", human), ("robot camera", robot)):
        found = serial in attached
        results.append(Result(label, found, True,
                              f"{serial} attached" if found
                              else f"{serial} not attached (attached: {', '.join(attached) or 'none'})"))
    return results


def attached_serials():
    from pypylon import pylon

    return [d.GetSerialNumber() for d in pylon.TlFactory.GetInstance().EnumerateDevices()]


def check_import(module):
    try:
        __import__(module)
    except Exception as error:  # ImportError or a broken native library
        return Result(f"python module {module}", False, True, f"{type(error).__name__}: {error}")
    return Result(f"python module {module}", True, True, "importable")


def run_checks(args, serial_lister=attached_serials, importer=check_import):
    results = [
        check_model_file("human pose model", args.human_pose),
        check_model_file("human cap model", args.human_cap),
        check_calibration("human calibration", args.human_calib),
        check_model_file("robot model", args.robot_model),
        check_model_file("toy model (optional)", args.robot_toy, required=False),
        check_calibration("robot calibration", args.robot_calib),
    ]
    if not args.skip_hardware:
        results += [importer("ultralytics"), importer("pypylon")]
        if results[-1].ok:
            try:
                attached = serial_lister()
            except Exception as error:
                results.append(Result("camera enumeration", False, True, str(error)))
            else:
                results += check_serials(args.human_serial, args.robot_serial, attached)
        else:
            results.append(Result("camera serials", False, True, "cannot enumerate without pypylon"))
    return results


def passed(results):
    # An absent optional item is ok (skipped); a present but invalid one blocks launch.
    return all(r.ok for r in results)


def parse(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--human-pose", required=True)
    parser.add_argument("--human-cap", required=True)
    parser.add_argument("--human-calib", required=True)
    parser.add_argument("--robot-model", required=True)
    parser.add_argument("--robot-toy", default="")
    parser.add_argument("--robot-calib", required=True)
    parser.add_argument("--human-serial", default="")
    parser.add_argument("--robot-serial", default="")
    parser.add_argument("--skip-hardware", action="store_true",
                        help="check files only (tests/development); the launcher never uses this")
    return parser.parse_args(argv)


def main(argv=None):
    results = run_checks(parse(argv))
    for r in results:
        status = "SKIPPED " if r.skipped else ("OK      " if r.ok else "FAILED  ")
        print(f"{status} {r.label}: {r.detail}")
    if passed(results):
        print("Rig assets OK.")
        return 0
    print("Rig assets NOT ready; fix the required items above.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
