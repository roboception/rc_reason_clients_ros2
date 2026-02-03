from setuptools import setup

package_name = "rc_reason_clients"

import subprocess
import sys
from pathlib import Path
from glob import glob


def generate_protos():
    proto_dir = Path("protos")
    # The python package directory is ./rc_reason_clients
    # We want to output to ./rc_reason_clients/generated
    out_dir = Path(package_name) / "generated"

    protos = glob(str(proto_dir / "*.proto"))
    if not protos:
        return

    # Ensure output directory exists
    out_dir.mkdir(parents=True, exist_ok=True)
    if not (out_dir / "__init__.py").exists():
        (out_dir / "__init__.py").touch()

    # Generate the code
    cmd = [
        sys.executable,
        "-m",
        "grpc_tools.protoc",
        f"-I{proto_dir}",
        f"--python_out={out_dir}",
        f"--grpc_python_out={out_dir}",
        *protos,
    ]
    # print(f"Generating protos with command: {' '.join(str(c) for c in cmd)}")
    subprocess.check_call(cmd)

    # Fix imports in generated grpc files for Python 3 relative imports
    # protoc usually generates 'import foo_pb2' which fails inside a package
    for grpc_file in out_dir.glob("*_grpc.py"):
        with open(grpc_file, "r") as f:
            content = f.read()

        # Dynamically fix imports for all protos found
        for proto_path in protos:
            proto_stem = Path(proto_path).stem
            old_import = f"import {proto_stem}_pb2"
            new_import = f"from . import {proto_stem}_pb2"
            content = content.replace(old_import, new_import)

        with open(grpc_file, "w") as f:
            f.write(content)


generate_protos()

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".generated"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Felix Ruess",
    maintainer_email="felix.ruess@roboception.de",
    description="Clients for interfacing with Roboception reason modules on rc_visard and rc_cube.",
    license="BSD",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "rc_april_tag_detect_client = rc_reason_clients.tagdetect:rc_april_tag_detect_client",
            "rc_qr_code_detect_client = rc_reason_clients.tagdetect:rc_qr_code_detect_client",
            "rc_silhouettematch_client = rc_reason_clients.silhouettematch:main",
            "rc_itempick_client = rc_reason_clients.pick:rc_itempick_client",
            "rc_boxpick_client = rc_reason_clients.pick:rc_boxpick_client",
            "rc_hand_eye_calibration_client = rc_reason_clients.hand_eye_calib:main",
            "rc_cadmatch_client = rc_reason_clients.cadmatch:main",
            "rc_load_carrier_client = rc_reason_clients.load_carrier:main",
            "rc_image_event_client = rc_reason_clients.image_event_client:main",
        ],
    },
)
