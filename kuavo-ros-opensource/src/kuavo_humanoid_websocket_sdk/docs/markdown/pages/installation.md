<a id="installation"></a>

# Installation

## Installation

There are two ways to install the Kuavo Humanoid SDK:

1. Install from PyPI (recommended)

   #### NOTE
   There are currently two versions of this SDK, the stable version and the beta version. Their differences are:
   - stable version: corresponding to the functionality provided by the master branch of [kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/).
   - Beta version: This version is more aggressive than the official version and also provides richer functionality, corresponding to the functionality provided by the beta branch of [kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/).

   #### NOTE
   Please be clear about which version you need to install. If your SDK version does not match kuavo-ros-opensource, some features may not be available.

   Install the latest **stable version** of Kuavo Humanoid SDK using pip:
   ```bash
   pip install kuavo-humanoid-sdk
   ```

   Install the latest **beta version** of Kuavo Humanoid SDK using pip:
   ```bash
   pip install --pre kuavo-humanoid-sdk
   ```
2. **Or** install in development mode

   Clone the repository and install in editable mode:
   ```bash
   git clone https://gitee.com/leju-robot/kuavo-ros-opensource.git
   cd kuavo-ros-opensource/src/kuavo_humanoid_sdk
   chmod +x install.sh
   ./install.sh
   ```

   This allows you to modify the source code and have the changes take effect immediately without reinstalling.

## Upgrading

> Before upgrading, you can check the currently installed version with:

> ```bash
> pip show kuavo-humanoid-sdk
> # Output:
> Name: kuavo-humanoid-sdk
> Version: 0.1.2
> ...
> ```

> #### NOTE
> If the version number contains the letter b, it indicates a beta version, e.g., Version: 0.1.2b113

> To upgrade from a **stable version** to the latest stable version:

> ```bash
> pip install --upgrade kuavo_humanoid_sdk
> ```

> To upgrade from a **beta version** to the latest stable version:

> ```bash
> pip install --upgrade --force-reinstall kuavo_humanoid_sdk
> # or
> pip uninstall kuavo_humanoid_sdk && pip install kuavo_humanoid_sdk
> ```

> To upgrade from a **stable/beta version** to the latest beta version:

> ```bash
> pip install --upgrade --pre kuavo_humanoid_sdk
> ```
