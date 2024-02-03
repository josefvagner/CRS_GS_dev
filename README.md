Repo for lora sx1280 CRS GS communication development

# GS (Fast api)
Fast api program can be deployed in Docker containter by using Makefile in RPI directory.

Available commands:
- make docker-build
  - build docker container
- make docker-run
  - run fast api container (to stop the container press CTRL+C)

Fast api runs on localhost so to see api response go to http://0.0.0.0:80.

# FC (Flight computer) - sx1280 library
Program for sx1280 module is developed in docker container and can be compiled with Makefile in FC/RpiPico/ directory.
