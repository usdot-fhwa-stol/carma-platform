/***
* Source: https://www.kenneth-truyers.net/2013/04/27/javascript-namespaces-and-modules/
***/

'use strict';

// Create the root namespace and making sure we're not overwriting it
var CarmaJS = CarmaJS || {};

// Create a general purpose namespace method
// This will allow us to create namespace a bit easier
CarmaJS.registerNamespace = function (namespace) {
    var nsparts = namespace.split(".");
    var parent = CarmaJS;

    // we want to be able to include or exclude the root namespace
    // So we strip it if it's in the namespace
    if (nsparts[0] === "CarmaJS") {
        nsparts = nsparts.slice(1);
    }

    // loop through the parts and create
    // a nested namespace if necessary
    for (var i = 0; i < nsparts.length; i++) {
        var partname = nsparts[i];
        // check if the current parent already has
        // the namespace declared, if not create it
        if (typeof parent[partname] === "undefined") {
            parent[partname] = {};
        }
        // get a reference to the deepest element
        // in the hierarchy so far
        parent = parent[partname];
    }
    // the parent is now completely constructed
    // with empty namespaces and can be used.
    return parent;
};

CarmaJS.registerNamespace("CarmaJS.Config");

CarmaJS.Config = (function () {
        //Private variables
        var ip = 'localhost'; //'192.168.88.10'; 192.168.32.146;

        //Private methods
        //Creating functions to prevent access by reference to private variables
        var getIP = function() {
            return ip;
        };

        //Public API
        return {
            getIP: getIP
        };
})();
