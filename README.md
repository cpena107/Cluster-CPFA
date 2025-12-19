# ClusterMap-CPFA-ARGoS

ARGoS (Autonomous Robots Go Swarming) is a multi-physics robot simulator. iAnt-ARGoS is an extension to ARGoS that implements the CPFA-ARGoS algorithm and provides a mechanism for performing experiments with iAnts.

### Updates
This project was forked from [MARSLab-UTRGV](https://github.com/MARSLab-UTRGV/CPFA-ARGoS) and updated. The project has been tested with the `CPFA_ClusterMap_random.xml` in the `/experiments` folder. You can follow the installation guide below to install and run the updated system.

---

### Quick Start Installation Guide

The CPFA-ARGoS system has two components: 
1. The **Cluster logic controllers** that implement the ClusterMap-CPFA algorithm in the ARGoS robot simulator.

You can run the ClusterMap-CPFA algorithm on ARGoS using OS X or Linux (see installation instructions below). 

---

### Installing ARGoS

ARGoS is available for Linux and Macintosh systems. It is currently not supported on Windows. Detailed installation instructions can be found on the [ARGoS Website](http://www.argos-sim.info/user_manual.php).

#### Linux Installation

1. [Download](http://www.argos-sim.info/core.php) the appropriate binary package for your Linux system.
2. In Terminal, run the following command in the directory of your installation file:
   - for Ubuntu and KUbuntu:

```bash
$ sudo dpkg -i argos3_simulator-*.deb
```

   - for OpenSUSE:

```bash
$ sudo rpm -i argos3_simulator-*.rpm
```

#### Macintosh Installation

1. Install Homebrew if you don’t already have it:

```bash
$ ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

2. Obtain the Homebrew Tap for ARGoS:

```bash
$ brew tap ilpincy/argos3
```

3. Install ARGoS with Homebrew:

```bash
$ brew install bash-completion qt lua argos3
```

4. To update ARGoS with Homebrew:

```bash
$ brew update
$ brew upgrade argos3
```

---

### Compiling and Running the CPFA in ARGoS

Once ARGoS is installed on your system, you can download the files in this repository, compile them for your system, and run the iAnt CPFA in ARGoS.

1. Pull the code from this repository.

2. From the terminal, use the `build.sh` script to compile the code:

```bash
$ ./build.sh
```

#### Using the CPFA evolver (optional)

> **Attention MARSLab**: Currently the evolver is designed to use the Moses MPI cluster which is UNM's computing cluster. This lab does NOT have access to the MPI cluster. This section only serves as a reference. Make sure `DBUILD_EVOLVER` is always set to `NO` in the build.sh. 

CPFA-ARGoS includes the CPFA evolver. This program uses a distributed version of ga-lib to evolve CPFA parameters. An example script for running `cpfa_evolver` is provided: `evolve_EXAMPLE.sh`.

CPFA evolver uses MPI to distribute ARGoS evaluations across a cluster. An example machine file (`moses_cluster`) specifies the hostnames of the MPI nodes to use and the number of processes to run on each node.

`evolve_EXAMPLE.sh` takes two arguments: the number of MPI processes to run and the machine file name for the MPI cluster.

Since the evolver relies on MPI packages that are not required for compiling the CPFA, compilation of the evolver is turned off by default.

To build the CPFA evolver, modify the `build.sh` script and change:

```bash
cmake -DBUILD_EVOLVER=NO ..
```

to:

```bash
cmake -DBUILD_EVOLVER=YES ..
```

The evolver takes an experiment XML file as an argument that specifies the simulation parameters. The CPFA genome in that experiment file is ignored and evolved parameters are used instead. Make sure visualization is turned off in this experiment file.

---

### 3. Running an Experiment
To run an experiment, launch ARGoS with the XML configuration file for your system:

```bash
$ argos3 -c experiments/CPFA_ClusterMap_random.xml
```

---

### Useful Links

| Description                                | Website                                                        |
|:-------------------------------------------|:---------------------------------------------------------------|
| Official ARGoS website and documentation   | http://www.argos-sim.info/                                     |
| Homebrew utility for Mac OSX installations | http://brew.sh/                                                |
| CMake utility information                  | http://www.cmake.org/documentation/                            |
