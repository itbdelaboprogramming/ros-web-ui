# ros-web-ui

## Description
This is a web interface for ROS packages. It is based on the [rosbridge_suite](http://wiki.ros.org/rosbridge_suite) package and uses AWS IoT Core to communicate with the ROS system.

## Installation
### Prerequisites
- [ROS](http://wiki.ros.org/ROS/Installation)
- [Docker](https://docs.docker.com/get-docker/)

### Certificate
To use AWS IoT Core, you need to create a certificate. You can follow the instructions [here](https://docs.aws.amazon.com/iot/latest/developerguide/create-device-certificate.html). After that, you need to download the certificate, the private key, and the root CA certificate. All of them should be in the `certs` folder and named `certificate.pem', 'private.pem.key', and 'rootCA.pem', respectively. Make sure to change the permissions of the private key file to 600 and all files on '.pem' format.

The Certificate project should look like this:
```bash
certs/
    certificate.pem
    private.pem.key
    rootCA.pem
```

### Build the Docker image
To build the Docker image, you need to run the following command:
```bash
cd Docker
docker build -t ros-container -f Dockerfile.rosbare . 
```

After building the Docker image, you need to run the container with the following command:
```bash
chmod +x docker.sh
./docker.sh
```

## Running the application
This application is divided into two parts: msd system and aws system.
### MSD System
To run the MSD system, you need to run the following command:
```bash
roslaunch msd700_webui_bringup bringup_msd.launch

roslaunch msd700_navigations msd700_navigation.launch open_rviz:=true use_simulator:=true
```

### AWS System
To run the AWS system, you need to run the following command:
```bash
roslaunch msd700_webui_bringup bringup_aws.launch
```


