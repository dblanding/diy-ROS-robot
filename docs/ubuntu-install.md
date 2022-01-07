# Installation and Configuration of Ubuntu and ROS on RasPi4
* [Install Ubuntu MATE 20.04 LTS on Raspberry Pi 4](https://linuxhint.com/install_ubuntu_mate_raspberry_pi_4/)
    * (On Windows) Write file `ubuntu-mate-20.04.1-desktop-arm64*raspi.img.xz` to 32 GB micro SDcard using Raspberry Pi Imager
    * Boot Pi from new SDcard and configure
* From Welcome screen -> Getting Started -> System Specifications -> Utilities
	* Install:
	    * System Profiler and Benchmark
	    * GParted
	    * Psensor
* `sudo apt update`
* `sudo apt upgrade`
* Add 'GitLab Markdown Viewer' extension to Firefox
    * None of the markdown viewer extensions worked.
    * Solution: Create a new mime type and update mime database
        * [How to get the Markdown Viewer addon of Firefox to work on Linux](https://superuser.com/questions/696361/how-to-get-the-markdown-viewer-addon-of-firefox-to-work-on-linux/1175837#1175837)
* Resolve `/usr/bin/env: ‘python’: No such file or directory` errors running python programs starting with `!#/usr/bin/env python`
    * Added symbolic link. `sudo ln -s python3 /usr/bin/python`
* Add robot (and raspi80) to /etc/hosts
* Install ROS: [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
* Configure ROS Environment: [Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
* Add line to ~/.bashrc : `source ~/catkin_ws/devel/setup.bash`
* [Install Arduino IDE and rosserial](arduino-install.md)

* Configure passwordless ssh to robot [How to setup SSH login without password on Linux systems](https://thelinuxgurus.com/how-to-setup-ssh-login-without-password/)
* Load codec to enable watching youtube videos. `sudo apt install libavcodec-extra`
* Still trying to get video and sound working. Need to edit a couple of files.
    * Google search took me [here](https://forums.raspberrypi.com/viewtopic.php?t=289126) which has a link [here](https://waldorf.waveform.org.uk/2020/ubuntu-desktops-on-the-pi.html)
* Install pip
    * `sudo apt update`
    * `sudo apt upgrade`
    * `sudo apt install python3-pip`
* `pip install RPi.GPIO`
* Install idle `sudo apt install idle3`
* Install python mathutils library `pip install mathutils` (used to normalize quaternion)
* Install Jupyter Notebook: [How to Install Jupyter Notebook on Ubuntu 20.04 / 18.04](https://speedysense.com/install-jupyter-notebook-on-ubuntu-20-04/)
    * Create ~/notebook/ directory (in home dir)
    * Cloned [Kalman-and-Bayesian-Filters-in-Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)/ under /notebook
    * To enter jupyter virtual env: `source notebook/jupyterenv/bin/activate`
    * Within jupyterenv, pip install the following:
        * scipy
        * numpy
        * matplotlib
        * filterpy
        * sympy
    * To start: `jupyter notebook`
* Install Caja Share (samba) allowing files on robot to be opened in Caja
* Install git -> git version 2.25.1 already installed!
    * `git config --global user.email "dblanding@gmail.com"`
    * `git config --global user.name "dblanding"`
    * [Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
    * [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
* Install [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) `sudo apt install ros-noetic-teleop-twist-keyboard`
* Install gmapping and navigation
    * `sudo apt install ros-noetic-gmapping`
    * `sudo apt install ros-noetic-navigation`

