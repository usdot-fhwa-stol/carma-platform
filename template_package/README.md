# template_package

This package serves as a template which can be copy and pasted to create a new ROS2 Node following CARMA Platform's default structure.
The contents of this package are not build-able on their own. There are a number of substitution parameters that must be replaced with the desired package name these are indicated with a ```<SUB><substitution name>``` indicator in the text. The following substitutions are used internally:
- ```<SUB><package_name>``` The new package name
- ```<SUB><year>``` The current year

To make the process easier a script is provided to create the new package with the desired package name and current year. 

# Creating a new ROS2 package for CARMA Platform

```bash
./carma_package <this template_package path> <package name> <path to new package parent folder>
```

For example, if you have a standard carma source checkout:

```bash
cd src/carma-platform/template_package
./carma_package . my_package ../
```

If successful you should see the new location printed.

```bash
my_package created at /workspaces/carma_ws/src/carma-platform/my_package
```

This new package can now be built like any other carma package.
