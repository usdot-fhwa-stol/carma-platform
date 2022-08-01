# template_package

This package serves as a template which can used to create a new ROS2 Node or Guidance Plugin Node following CARMA Platform's default structure.
The contents of this package are not build-able on their own. There are a number of substitution parameters that must be replaced with the desired package name these are indicated with a ```<SUB><substitution name>``` indicator in the text. The following substitutions are used internally:

- ```<SUB><package_name>``` The new package name
- ```<SUB><year>``` The current year
- ```<SUB><base_node_dep>``` The dep to use as a base node

To make the process easier a script is provided to create the new package of the desired type with the desired package name and current year. 

## Creating a new ROS2 package for CARMA Platform

```bash
./carma_package <this template_package path> <package name> <path to new package parent folder> <optional plugin type>
# If set, <optional plugin type> should be one of strategic, tactical, or control. If unset, then not a plugin"
```

For example, if you have a standard carma source checkout and want to make a regular node:

```bash
cd src/carma-platform/template_package
./carma_package . my_package ../
```

If successful you should see the new location printed.

```bash
my_package created at /workspaces/carma_ws/src/carma-platform/my_package
```

To make a new Guidance Plugin, you must specify the plugin type:

```bash
cd src/carma-platform/template_package
./carma_package . my_package ../ strategic
```

This new package can now be built like any other carma package.
