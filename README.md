# CloudCompare plugin for mapit IO

This plugin enables [CloudCompare](https://github.com/CloudCompare/CloudCompare) to open a [mapit](https://github.com/MASKOR/mapit) repository.
For this one must create a *.cc-mapit file within the mapit repo folder with the structre

```
--frame_id
<the frame id>
--workspace
<the workspace name>
--files
<file to be loaded #1>      # this must be contained in the filename in mapit
<file to be loaded #2>      # and can be read as .*<filename>.*
<file to be loaded #3>
<file to be loaded #4>
```

## Install

1. Clone this repo into CloudCompare at ```plugins/3rdParty/```
1. Activate this plugin in cmake, by enableing ```INSTALL_QMAPIT_IO_PLUGIN```
1. Compile and install CloudCompare