CarmaJS.registerNamespace("CarmaJS.WidgetFramework");

CarmaJS.WidgetFramework = (function () {
        //Private variables

        //Private methods
        //Creating functions to prevent access by reference to private variables
        //Load javascript file references.
        //https://stackoverflow.com/questions/13121948/dynamically-add-script-tag-with-src-that-may-include-document-write
        //NOTE: appending to document.head doesn't seem to work, need to have this callback to fully load the script.
        //Can parse multiple files. e.g. scriptLoader(['a.js','b.js'], functionname)
        var scriptLoader = function (scripts, callback) {
            var count = scripts.length;

            function urlCallback(url) {
                return function () {
                    //** DO NOT REMOVE THIS console.log, somehow this makes a difference on loading the script properly.
                    console.log(url + ' was loaded (' + --count + ' more scripts remaining).');

                    if (count < 1) {
                        callback();
                    }
                };
            }

            function loadScript(url) {

                var s = document.createElement('script');
                s.setAttribute('src', url);
                s.onload = urlCallback(url);
                document.head.appendChild(s);

            }

            for (var script of scripts) {
                loadScript(script);
            }
        };


        var showSelectedWidgets = function () {
            //Get selected widgets from divWidgetOptionsList
            var divWidgetOptionsList = document.getElementById('divWidgetOptionsList');
            var selectedWidgets = getCheckboxesSelected(divWidgetOptionsList);

            //Update the parameter to note selected widget
            selectedWidgets.forEach(function (widget) {

               //Update plugin list
                var pluginsActivated = sessionStorage.getItem('pluginsActivated');

               //alert('selectedWidgets: pluginsActivated: ' + pluginsActivated); //Show all widgets in the array.

                if (pluginsActivated == null || pluginsActivated == 'undefined')
                {
                    console.log ('showSelectedWidgets: No plugins activated.');
                    return;
                }

               pluginsActivated = JSON.parse(pluginsActivated);

              for (var i in pluginsActivated) {

                //alert('pluginsActivated[i].id: ' + pluginsActivated[i].id + '; widget.id: ' + widget.id);

                if (pluginsActivated[i].id == widget.id.substring(2, widget.id.length)) {
                   pluginsActivated[i].isWidgetShown = true;
                   //alert ('found!');
                   break; //Stop this loop, we found it!
                }
              }

               sessionStorage.setItem('pluginsActivated', JSON.stringify(pluginsActivated));
            });//ForEach

           //Load widgets
           this.loadWidgets();

        };

        //Show user selection of widgets based on Plugins that have been activated.
        //pluginId is currently is the PluginName concatenated with _ and removing "Plugin" at the end.
        var showWidgetOptions = function() {

            //Display the list of widgets
            var divWidgetOptions = document.getElementById('divWidgetOptions');
            divWidgetOptions.style.display = 'block';

            var pluginsActivated = sessionStorage.getItem('pluginsActivated');

            if (pluginsActivated == null || pluginsActivated == 'undefined')
            {
                console.log ('No widgets activated.');
                return;
            }

            //alert(pluginsActivated); //Show all widgets in the array.

            pluginsActivated = JSON.parse(pluginsActivated);

            var divWidgetOptionsList = document.getElementById('divWidgetOptionsList');
            divWidgetOptionsList.innerHTML = '';

            //Loop thru and create checkboxes
            pluginsActivated.forEach(function (plugin) {

                //Create the checkbox based on the plugin properties.
                createCheckboxElement(divWidgetOptionsList, plugin.id, plugin.title, pluginsActivated.length, 'groupWidgets', pluginsActivated.isWidgetShown, false, 'CarmaJS.WidgetFramework.activateWidget');
            });
        };

        var activatePlugin = function(id, title) {

                //alert('WidgetFramework.activateWidget - Plugin activated:' + id)
                //e.g. WidgetFramework.activateWidget - Plugin activated: cbNegotiation_Receiver_Plugin&1_0_0

                //Save the id and title for creating checkboxes.
                var cbId = id.replace('Plugin', 'Widget').substring(0,id.replace('Plugin', 'Widget').length);
                var cbTitle = title.replace('Plugin', 'Widget');

                //Add the plugin namespace and folder path into the array for loading.
                //Widget Namespace should come from the Plugin.name without "Plugin" at the end, and without spaces.
                //e.g. "Speed Harmonization Plugin" widget namespace should be CarmaJS.WidgetFramework.SpeedHarmonization
                //e.g. CarmaJS.WidgetFramework.RouteFollowing & CarmaJS.WidgetFramework.Cruising
                var widgetNamespace = 'CarmaJS.WidgetFramework.' + id.substring(0, id.indexOf('_Plugin')).replace('cb','').replace('_','');

                //Widget install path should come from the Plugin.name without the "Plugin", and replacing the spaces with underscore(s).
                //e.g. "Speed Harmonization Plugin" widget folder path should be speed_harmonization
                //e.g. widgets/route_following
                var widgetInstallPath = 'widgets/' + id.substring(0, id.indexOf('_Plugin')).replace('cb','').toLowerCase(); //negotiation_receiver

                var pluginItem = {id: cbId, title: cbTitle,  namespace: widgetNamespace, folderpath: widgetInstallPath, isWidgetShown: false};
                var pluginsActivated = sessionStorage.getItem('pluginsActivated');

                if (pluginsActivated != 'undefined' && pluginsActivated != null && pluginsActivated != '') {
                     pluginsActivated = JSON.parse(pluginsActivated);
                     pluginsActivated.push(pluginItem);
                     sessionStorage.setItem('pluginsActivated', JSON.stringify(pluginsActivated));
                }
                else{
                    pluginsActivated = [];
                    pluginsActivated.push(pluginItem);
                    sessionStorage.setItem('pluginsActivated', JSON.stringify(pluginsActivated));
                }

                //alert(pluginsActivated.); //Show all widgets in the array.
        };

        var loadWidgets = function(){

           //Get plugin list
           var pluginsActivated = sessionStorage.getItem('pluginsActivated');

           if (pluginsActivated == null || pluginsActivated == 'undefined')
           {
               console.log ('No widgets to load.');
               return;
           }

           //alert('loadWidgets.pluginsActivated: ' + pluginsActivated);

           //alert(pluginsActivated); //Show all widgets in the array.
           pluginsActivated = JSON.parse(pluginsActivated);

           //Check if results.
           var result = $.grep(pluginsActivated, function(e){ return e.isWidgetShown == true; });

            if (result.length == 0) {
              // not found, show OptionsList
              //showWidgetOptions();
              console.log ('No widgets to load');
              return;
            } else if (result.length == 1) {
              // access the foo property using result[0].foo
              console.log('1 widget to load');

            } else {
              // multiple items found
              console.log('multiple widgets to load');
            }

          //Hide the list of widgets
          var divWidgetOptions = document.getElementById('divWidgetOptions');
          divWidgetOptions.style.display = 'none';

           //Hide the list of widgets
          //  var divWidgetOptions = document.getElementById('divWidgetOptions');
           // divWidgetOptions.style.display = 'none';

            //Loop thru the active widgets and load each css, js and call it's loadCustomWidget().
            //Check if at least one of the expected file exists.
            //namespace: widgetNamespace, folderpath: widgetInstallPath};
            result.forEach(function (plugin) {

                //alert('plugin.id: ' + plugin.id + '; plugin.title: ' + plugin.title + '; plugin.isWidgetShown: ' + plugin.isWidgetShown );

                if (plugin.isWidgetShown == false)
                    return;

                var widgetNamespace = plugin.namespace;
                var widgetInstallPath = plugin.folderpath;

                if (widgetNamespace== null || widgetInstallPath == null)
                {
                    console.log('Widget namespace or install path is null.');
                    return;
                }

                var cssFilePath = widgetInstallPath + '/widget.css';
                var jsFilePath = widgetInstallPath + '/widget.js';

                $.ajax({
                     url: jsFilePath,
                     type:'HEAD',
                     error: function()
                     {
                         //file not exists
                         //TODO: In chrome, even with statusCode or error handling, the HTTP 404 (Failed to Load) error still shows separately
                         console.log('loadWidgets: Widget file does NOT exist: ' + jsFilePath );
                         return false;
                     },
                     success: function()
                     {
                        //console.log('cssFilePath: ' + cssFilePath);

                        //console.log('loadWidgets: Widget file DOES exist: ' + jsFilePath );
                        //1) Load css
                        var link = document.createElement('link');
                        link.setAttribute('rel', 'stylesheet');
                        link.setAttribute('type', 'text/css');
                        link.setAttribute('href', cssFilePath);
                        document.getElementsByTagName('head')[0].appendChild(link);

                        //2) Load JS
                        //console.log('jsFilePath1: ' + jsFilePath);
                        scriptLoader([jsFilePath],function()
                        {
                             // now you can use the code from loaded script files.
                             eval(widgetNamespace + '.loadCustomWidget($("#divWidgetArea"));');
                        });

                        return true;
                     }
                });
            });
        };

        var  closeWidgets = function(){
            CarmaJS.WidgetFramework.Cruising.closeWidget();
            console.log('widgetfw.closeWidgets!');
        };

        var onGuidanceEngaged = function(){
            CarmaJS.WidgetFramework.RouteFollowing.onGuidanceEngaged();
            console.log('widgetfw.onGuidanceEngaged!');

        };

        var onRouteSelection = function(){
            CarmaJS.WidgetFramework.RouteFollowing.onRouteSelection();
            console.log('widgetfw.onRouteSelection!');
        };

        var onRefresh = function () {
             //sessionStorage.removeItem('widgetsActivated');
             sessionStorage.removeItem('pluginsActivated');
        };

        //TODO: Need to hide and show widget here and update array to have isWidgetShown = false.
        var activateWidget = function(id) {
            console.log('activateWidget: ' + id);
        };

        //Public API
        return {
            showWidgetOptions: showWidgetOptions,
            showSelectedWidgets: showSelectedWidgets,
            loadWidgets: loadWidgets,
            activatePlugin: activatePlugin,
            activateWidget: activateWidget,
            closeWidgets: closeWidgets,
            onGuidanceEngaged: onGuidanceEngaged,
            onRouteSelection: onRouteSelection,
            onRefresh: onRefresh
        };
})();