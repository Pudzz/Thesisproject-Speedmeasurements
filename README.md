# Thesisproject-Speedmeasurements
Contains code from four different Python Modules, as well as Maya's C ++ API used for speed measurements to transfer animations between two skeleton hierarchies.

To test the scripts on your own computer and save data about the speeds, the
path to the respective text file needs to be adjusted in the code. In the implementation
of the C++ API, you also need to include the local from Maya, as well as the
.lib and .bin files for the .mll file to be able to compile and be created. Then the
respective skeleton root joints must be renamed as sourceRoot and targetRoot and
be marked in the order source and target, before the .mll file is to be run via plugin.
