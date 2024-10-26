#!/usr/bin/env python3
# Run example by downloading proper dataset


import pathlib
import requests
import zipfile
import tqdm
import logging
import subprocess
from typing import Literal


def download(base: pathlib.Path, name: str, urls: dict[str, str]):
    """
    Download file from urls[name], then save it to base/name.zip
    If base/name.zip already exists, no download occure.
    """

    path = base / (name + ".zip")

    base.mkdir(exist_ok=True, parents=True)
    if path.exists():
        logging.info("file already downloaded: " + str(path))
        return

    url = urls[name]
    logging.info("start downloading file from " + url)
    response = requests.get(url, stream=True)
    size = int(response.headers.get("content-length"))
    with open(path, "wb") as out:
        iterable = response.iter_content(chunk_size=1024)
        for chunk in tqdm.tqdm(iterable, total=size // 1024, unit="KB"):
            out.write(chunk)
    logging.info("download finished")


def unzip(base: pathlib.Path, name: str):
    """
    Unzip base/name.zip and save it to base/name directory.
    If base/name already exists, no uncompression occure.
    """

    zip_path = base / (name + ".zip")
    unzip_path = base / name

    if unzip_path.exists():
        return

    with zipfile.ZipFile(zip_path, "r") as zip:
        zip.extractall(unzip_path)


def create_example_args(
    base: pathlib.Path,
    method: Literal["EuRoC"],
    camera: Literal["Stereo"],
    inertial: bool,
    dataset: str,
) -> [str]:
    dataset_path = base / dataset
    if method == "EuRoC":
        if camera == "Stereo":
            if inertial:
                raise NotImplementedError("inertial is not supported yet")
            else:
                return [
                    "./build/Examples/Stereo/stereo_euroc",
                    "./Vocabulary/ORBvoc.txt",
                    "./Examples/Stereo/EuRoC.yaml",
                    str(dataset_path),
                    "./Examples/Stereo/EuRoC_TimeStamps/" + dataset + ".txt",
                    "dataset-" + dataset + "-stereo",
                ]


def run_example(
    args: [str],
    method: Literal["EuRoC"],
    camera: Literal["Stereo"],
    inertial: bool,
    dataset: str,
):
    out_log_path = pathlib.Path("./example.out.log")
    err_log_path = pathlib.Path("./example.err.log")

    logging.info("start running example")
    logging.info("method: " + method)
    logging.info("camera: " + camera)
    logging.info("use_inertial: " + str(inertial))
    logging.info("dataset: " + dataset)

    with open(out_log_path, "w") as out, open(err_log_path, "w") as err:
        subprocess.run(args, stdout=out, stderr=err)

    logging.info("finished running example")
    logging.info("latest example stdout log saved: " + str(out_log_path))
    logging.info("latest example stderr log saved: " + str(err_log_path))


def request_dataset_from_user() -> str:
    # TODO: Require user input
    return "MH04"


def request_camera_from_user() -> str:
    # TODO: Require user input
    return "Stereo"


def request_inertial_from_user() -> bool:
    # TODO: Require user input
    return False


def create_urls() -> dict[str, str]:
    head = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/"
    urls = {}
    urls["MH01"] = head + "machine_hall/MH_01_easy/MH_01_easy.zip"
    urls["MH02"] = head + "machine_hall/MH_02_easy/MH_02_easy.zip"
    urls["MH03"] = head + "machine_hall/MH_03_medium/MH_03_medium.zip"
    urls["MH04"] = head + "machine_hall/MH_04_difficult/MH_04_difficult.zip"
    urls["MH05"] = head + "machine_hall/MH_05_difficult/MH_05_difficult.zip"
    urls["V101"] = head + "vicon_room1/V1_01_easy/V1_01_easy.zip"
    urls["V102"] = head + "vicon_room1/V1_02_medium/V1_02_medium.zip"
    urls["V103"] = head + "vicon_room1/V1_03_difficult/V1_03_difficult.zip"
    urls["V201"] = head + "vicon_room1/V2_01_easy/V2_01_easy.zip"
    urls["V202"] = head + "vicon_room1/V2_02_medium/V2_02_medium.zip"
    urls["V203"] = head + "vicon_room1/V2_03_difficult/V2_03_difficult.zip"
    return urls


def main():
    logging.basicConfig(
        level=logging.INFO, format="[%(levelname)s] %(asctime)s %(message)s"
    )

    # Constants
    method = "EuRoC"
    base = pathlib.Path("./dataset")
    urls = create_urls()

    # Parameters for example
    camera = request_camera_from_user()
    inertial = request_inertial_from_user()
    dataset = request_dataset_from_user()

    try:
        download(base, dataset, urls)
        unzip(base, dataset)
        args = create_example_args(base, method, camera, inertial, dataset)
        run_example(args, method, camera, inertial, dataset)
    except NotImplementedError as e:
        logging.error(f"{e}")


if __name__ == "__main__":
    main()
