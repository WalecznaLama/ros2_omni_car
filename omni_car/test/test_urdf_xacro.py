import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


def test_urdf_xacro():
    # General Arguments
    description_package = "omni_car"
    description_file = "omni_car.xacro"

    description_file_path = os.path.join(
        get_package_share_directory(description_package), "urdf", description_file
    )

    (_, tmp_urdf_output_file) = tempfile.mkstemp(suffix=".urdf")

    # Compose `xacro` and `check_urdf` command
    xacro_command = (
        f"{shutil.which('xacro')}" f" {description_file_path}" f" > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

    # Try to call processes but finally remove the temp file
    try:
        xacro_process = subprocess.run(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert xacro_process.returncode == 0, " --- XACRO command failed ---"

        check_urdf_process = subprocess.run(
            check_urdf_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert (
            check_urdf_process.returncode == 0
        ), "\n --- URDF check failed! --- \nYour xacro does not unfold into a proper urdf robot description. Please check your xacro file."

    finally:
        os.remove(tmp_urdf_output_file)


if __name__ == "__main__":
    test_urdf_xacro()
