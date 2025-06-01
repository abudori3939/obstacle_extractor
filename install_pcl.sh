#!/bin/bash

# Update package lists
sudo apt-get update

# Install PCL and its dependencies
sudo apt-get install -y libpcl-dev pcl-tools
