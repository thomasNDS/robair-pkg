Robair ROS stack with interface
################


Get sources
===========

::

    roscd
    git clone https://github.com/thomasNDS/robair-pkg.git

Install dependencies
====================

::
    sudo pip install requirement.txt
    sudo ./setup-debs.sh



Run on Robot
============

::

    roscd robair-ros-pkg
    rosmake


::

    make robair


Displays information about ROS topics with rostopic command-line  

::

    rostopic echo /info/battery



Launch Web server
=================

Launch the reservation platform at:
:: https://github.com/thomasNDS/robAirManager.git

Launch the serverWeb node
:: rosrun robair_driver serverWeb




