#!/bin/bash
# Create a conda environment
ISAAC_SIM_PATH=$HOME/.local/share/ov/pkg/isaac-sim-2023.1.1
ISAAC_PYTHON_VERSION="3.10.13"
echo "ISAAC_PYTHON_VERSION = ${ISAAC_PYTHON_VERSION}"

source $(conda info --base)/etc/profile.d/conda.sh
ENV_NAME="nio3"
conda create -y -n "${ENV_NAME}" "python=${ISAAC_PYTHON_VERSION}"

conda activate ${ENV_NAME}
mkdir -p ${CONDA_PREFIX}/etc/conda/activate.d
CONDA_ACTIVATE=${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh

# Set variable
echo "export ISAAC_SIM_PATH=$HOME/.local/share/ov/pkg/isaac-sim-2023.1.1">>${CONDA_ACTIVATE}
echo "source ${ISAAC_SIM_PATH}/setup_conda_env.sh" >> ${CONDA_ACTIVATE}
echo "source ${ISAAC_SIM_PATH}/setup_python_env.sh" >> ${CONDA_ACTIVATE}

mkdir -p ${CONDA_PREFIX}/etc/conda/deactivate.d
CONDA_DEACTIVATE=${CONDA_PREFIX}/etc/conda/deactivate.d/env_vars.sh
echo "unset ISAAC_PATH" >> ${CONDA_DEACTIVATE}
echo "unset CARB_APP_PATH" >> ${CONDA_DEACTIVATE}
echo "unset EXP_PATH" >> ${CONDA_DEACTIVATE}
echo "unset PYTHONPATH" >> ${CONDA_DEACTIVATE}


pip install --upgrade pip
pip install pre-commit
pip install -e .[dev]
if ! command -v pre-commit &> /dev/null
then
    echo "pre-commit could not be found"
    exit 1
fi


pre-commit install


conda deactivate
echo -e "\nnio successfully installed!"
