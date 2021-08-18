######
Docker
######

This directory contain the files required to build the docker image for deploying the MTMount CSC.
The deployable artifact is built on top of the Telescope and Site deployment environment.
The image build instructions are specified in the `Dockerfile`.
A second file, `setup.sh`, controls how the software is loaded when the image runs.

The build script has four arguments.

* `deploy_tag`: The base image tag. This will basically define the version of
  salobj used and a couple other environment details.
* `mtmount`: The conda version of the ts_mtmount package.
* `idl`: The conda version of the ts_idl package.
* `config`: The git version of the configuration package.

To build the image manually run the following, selecting appropriate values
for each build argument:

```
docker build -t mtmount --build-arg mtmount=0.1.dev68+gf5d71b5 --build-arg deploy_tag=salobj_5.10.0 --build-arg idl=1.1.3_5.0.0
```
