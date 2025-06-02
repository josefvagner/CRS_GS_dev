# Cansat FSW RP2040 development

## Docker installation for Windows with WSL2
**Turn on Docker Desktop WSL 2**
- Ensure you are in WSL2 mode, run `wsl.exe -l -v` in Windows terminal.
- If not:
  1. Upgrade the Linux distro to v2, run: `wsl.exe --set-version (distro name) 2`
  2. Set v2 as the default version for future installations, run: `wsl.exe --set-default-version 2`
- Download and install the latest version of [Docker Desktop for Windows](https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe)
- Follow the usual installation instructions to install Docker Desktop. 
- When installing make sure the **Use WSL 2 based engine.** the option is turned on (it should be by default)
- If you had docker installed previously and want to make sure:
  1. Start Docker Desktop from the Windows Start menu.
  2. Navigate to Settings.
  3. From the General tab, select **Use WSL 2 based engine.**
  4. Select Apply & Restart.

**Enabling Docker support in WSL 2 distros**
- Start Docker Desktop from the Windows Start menu.
- When Docker Desktop starts, go to Settings > Resources > WSL Integration.
- Select distro on which you want to enable docker.
- Select **Apply & Restart**.

## Container installation for macOS with Rancher Desktop

```shell
brew install --cask rancher
```

---

## Start:

- **_Pull from GitHub_** - Pull the last version of repository from GitHub.

- **_Build docker for development_** - in this folder run `make docker-build` (you need to run this only when the docker file changes, or you delete the old docker image)

- **_Run docker_** - in this folder run `make docker-run`. It will run the docker image and move you to its shell. 

## Connect from VS code:
- make sure you have **Dev containers** extension installed
- navigate to **Remote explorer** tab, select **Dev containers** from the drop-down menu and open **rpi/pico/apps** container folder.
- This whole folder is auto-synchronized with docker, so every edit you do on the docker is mirrored into this directory on your local machine.

## Development:

- **_Building_** - In apps directory run `make`, this will create **build** directory and run CMakeLists.txt.
Then you can navigate to **apps/build/{app name}/** and run `make all` to compile your app and generate .uf2 files.
(Optionally you can run `make` in **apps/{app name})

- **_Adding new apps and files_** - If you want to add new app to apps use directory dummyApp which is a template with CMakeLists.txt file, src and include directory. Copy the dummyApp and rename it. Add your app for compilation in global CMakeLists file in this folder (add_subdirectory at line 18). Then edit CMakeLists file in your app by provided instructions in the file. Create files in src and include directory and compile.

- **_Other than your app is compiling with error_** - edit the problematic directory from root CMakeLists file, delete build directory, and build again. Don't push this to GitHub.

- **_Running program on RPi Pico_** - Compiled .u2f files are located inside build directory under your app directory. 
You can open the folder from WSL since it is synchronized. Hold `BOOTSEL` button on RPi Pico when plugging it in and then copy the .u2f file to Rpi directory that shows as connected flash disk.

- **_Permissions_** - In case you are not allowed to copy / delete files in **apps* directory run `chmod -R 777 *` inside **apps** directory.

## Ending:

- **_Stop docker_** - Disconnect from the docker and then in the shell that opens after you run the docker run `exit`. This will stop the running docker.

- **_Push to GitHub_** - Write edited data to GitHub just like you normally do. The files are stored on your local drive.

## Global libraries:

The provided global directory should be used for programs (libraries) that are needed in more than one app.
For example, the CAN libraries.