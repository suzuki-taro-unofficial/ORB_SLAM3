#!/usr/bin/bash

# Almost same as select, but return default value if only enter pressed.
# $1: prompt message
# $2: reference to list of options user select
# $3: default index of the list
function select_with_default {
    local message="$1"
    local -n options_ref="$2"
    local default="$3"
    while true; do
        len="${#options_ref[@]}"
        digits="${#len}"
        for i in "${!options_ref[@]}"; do
            printf "%${digits}d) %s\n" "$((i+1))" "${options_ref[i]}"
        done
        read -p "${message} (default $((default+1))): " choice
        if [[ -z "$choice" ]]; then
            return $default
        elif [[ $choice =~ ^[0-9]+$ && $((choice-1)) -lt $len ]]; then
            return $((choice-1))
        else
            echo "invalid selection. try again."
        fi
    done
}

function init_urls {
    local -n urls_ref="$1"
    local head="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
    urls_ref[MH01]="${head}/machine_hall/MH_01_easy/MH_01_easy.zip"
    urls_ref[MH02]="${head}/machine_hall/MH_02_easy/MH_02_easy.zip"
    urls_ref[MH03]="${head}/machine_hall/MH_03_medium/MH_03_medium.zip"
    urls_ref[MH04]="${head}/machine_hall/MH_04_difficult/MH_04_difficult.zip"
    urls_ref[MH05]="${head}/machine_hall/MH_05_difficult/MH_05_difficult.zip"
    urls_ref[V101]="${head}/vicon_room1/V1_01_easy/V1_01_easy.zip"
    urls_ref[V102]="${head}/vicon_room1/V1_02_medium/V1_02_medium.zip"
    urls_ref[V103]="${head}/vicon_room1/V1_03_difficult/V1_03_difficult.zip"
    urls_ref[V201]="${head}/vicon_room1/V2_01_easy/V2_01_easy.zip"
    urls_ref[V202]="${head}/vicon_room1/V2_02_medium/V2_02_medium.zip"
    urls_ref[V203]="${head}/vicon_room1/V2_03_difficult/V2_03_difficult.zip"
}

function init_base {
    local -n base_ref="$1"
    base_ref="./dataset"
}

function accept_method {
    # TODO: Other method
    local -n method_ref="$1"
    local options=("EuRoC")
    select_with_default "select method" options 0
    method_ref="${options[$?]}"
}

function accept_dataset {
    local -n dataset_ref="$1"
    local options=("MH01" "MH02" "MH03" "MH04" "MH05" "V101" "V102" "V103" "V201" "V202" "V203")
    select_with_default "select dataset" options 0
    dataset_ref="${options[$?]}"
}

function accept_inertial {
    local -n inertial_ref="$1"

    while true; do
        read -p "use inertial? (y/n) (default n): " input
        if [[ "${input}" = "y" || "${input}" = "Y" ]]; then
            inertial_ref=true
            break
        elif [[ "${input}" = "n" || "${input}" = "N" ]]; then
            inertial_ref=false
            break
        elif [[ -z "${input}" ]]; then
            inertial_ref=false
            break
        else
            echo "invalid input. try again."
        fi
    done
}

function accept_camera {
    local -n camera_ref="$1"
    local options=("Stereo" "RGB-D" "Monocular")
    select_with_default "select camera" options 0
    camera_ref="${options[$?]}"
}

function download_dataset {
    local base="$1"
    local dataset="$2"
    local -n urls_ref="$3"

    local zip_path="${base}/${dataset}.zip"

    mkdir -p $base
    if [[ -f $zip_path ]]; then
        echo "${dataset} already downloaded"
    else
        echo "download ${dataset}"
        if curl ${urls_ref[$dataset]} -# -o $zip_path; then
            echo "download finished"
        else
            echo "download failed"
            exit 1
        fi
    fi
}

function unzip_dataset {
    local base="$1"
    local dataset="$2"

    local zip_path="${base}/${dataset}.zip"
    local unzip_path="${base}/${dataset}"

    if [[ -d $unzip_path ]]; then
        echo "${dataset} already uncompressed"
    else
        echo "uncompress ${dataset}"
        if unzip $zip_path -d $unzip_path; then
            echo "uncompression finished"
        else
            echo "uncompression failed"
            exit 1
        fi
    fi
}

function run_example {
    local base="$1"
    local dataset="$2"
    local inertial="$3"
    local camera="$4"

    echo "running example"
    echo "dataset: ${dataset}"
    echo "inertial: ${inertial}"
    echo "camera: ${camera}"

    local dataset_path="${base}/${dataset}"
    if [[ "${camera}" = "Stereo" ]]; then
        if "${inertial}"; then
            echo "not supported yet"
            return
        else
            trap SIGSEGV
            ./build/Examples/Stereo/stereo_euroc \
                ./Vocabulary/ORBvoc.txt \
                ./Examples/Stereo/EuRoC.yaml \
                $dataset_path \
                ./Examples/Stereo/EuRoC_TimeStamps/${dataset}.txt \
                dataset-${dataset}-stereo \
                > example.out.log 2> example.err.log
        fi
    elif [[ "${camera}" = "RGB-D" ]]; then
        echo "not supported yet"
        return
    elif [[ "${camera}" = "Monocular" ]]; then
        echo "not supported yet"
        return
    fi

    echo "finished running example"
    echo "latest example stdout log saved: example.out.log"
    echo "latest example stderr log saved: example.err.log"
}

declare -A urls
init_urls urls

declare base
init_base base

declare method
accept_method method

declare dataset
accept_dataset dataset

declare inertial
accept_inertial inertial

declare camera
accept_camera camera

download_dataset $base $dataset urls
unzip_dataset $base $dataset
run_example $base $dataset $inertial $camera
