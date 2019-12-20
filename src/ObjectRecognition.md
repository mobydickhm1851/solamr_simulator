# Object Recognition Using LINE-MOD

Basically, all you need is following [this tutorial][ork]. 
However, few problems occured while installing using apt-install. (DISTRO: kinetic)

First, let's install ORK packages

'''
    sudo apt install ros-kinetic-object-recognition-*
'''
which should include:
'''
    capture, core, msgs, reconstruction, ros, ros-visualization, tod, transparent-objects.
'''
By comparing these pkgs with waht's in the [tutorial][ork_install] of building from source, these pkgs are missing:
'''
    *linemod*, ork_renderer
'''

so we'll have to build these pkgs from source:
First, go to the workspace, which I named "linemod" and placed in solamr_simulator/src.
Then clone the pkgs

'''
    git clone http://github.com/wg-perception/linemod
    git clone http://github.com/wg-perception/ork_renderer
'''

Then build it using 'catkin_make'.

Where, build error like this might occur
'''
    fatal error: GL/osmesa.h: No such file or directory...
'''

This is due to the missing of pkg 'libosmesa6'
So just install this pkg
'''
    sudo apt install libosmesa6-dev
'''

Then 'catknin_make' again.


### References:

- The paper of [LINE-MOD][linemod].
- [Discussion][dis] of LINE-MOD.


[ork]:https://wg-perception.github.io/ork_tutorials/tutorial03/tutorial.html
[ork_install]:http://wg-perception.github.io/object_recognition_core/install.html#install
[linemod]:http://campar.in.tum.de/pub/hinterstoisser2011linemod/hinterstoisser2011linemod.pdf
[dis]:https://zhuanlan.zhihu.com/p/35683990
