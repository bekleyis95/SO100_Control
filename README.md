# SO100 Control

6-DOF robot control with inverse kinematics and trajectory generation.

## Setting Up the Environment

Follow these steps to set up the project environment:

### 1. Create a Conda Environment
Ensure you have [Conda](https://docs.conda.io/en/latest/) installed. Then, create a new Conda environment:

```bash
conda create -n so100_control python=3.10 -y
conda activate so100_control
```

### 2. Install Dependencies with Poetry
Ensure you have [Poetry](https://python-poetry.org/) installed. Then, install the project dependencies:

```bash
poetry install
```

This will install all required dependencies, including the `lerobot` package from the specified Git repository.

### 3. Verify Installation
Run the following command to verify that everything is set up correctly:

```bash
poetry check
```

You are now ready to use the SO100 Control project!
