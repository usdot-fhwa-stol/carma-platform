read -p 'WARNING this script deletes all the files under opt/carma/src and var/www/html/, are you sure you want to continue? <y/n> ' prompt
if [[ $prompt == 'n' || $prompt == 'N' || $prompt == 'no' || $prompt == 'No' ]]
then
  echo 'You answered' $prompt ', exiting...' 
  exit 0
fi

read -sp 'What Branch Path?: ' branchvar
echo
echo 'Branch: ' $branchvar 
read -p 'Is this the correct branch? <y/n> ' prompt2

if [[ $prompt2 == "n" || $prompt2 == "N" || $prompt2 == "no" || $prompt2 == "No" ]]
then
  echo 'You answered' $prompt2 ', exiting to start from the top.' 
  exit 0
fi

echo 'Deleting files under /opt/carma...'
cd /opt/carma
sudo rm -r /opt/carma/src && sudo rm -r /opt/carma/devel && sudo rm -r /opt/carma/build

echo 'Creating new src folder and git clone: ' $branchvar
mkdir src
cd src
git clone https://github.com/fhwa-saxton/CarmaPlatform --branch $branchvar

echo 'Updating routes folder'
if [ ! -d "routes" ]; then # if routes folder does not exist then make it
  mkdir /opt/carma/routes
fi
sudo cp -R /opt/carma/src/CarmaPlatform/carmajava/route/src/test/resources/routefiles/* /opt/carma/routes/

echo 'Deleting files from html folder and copying new files ...'
cd /var/www/html
sudo rm -r *
sudo cp -R -r /opt/carma/src/CarmaPlatform/website/* /var/www/html/

sudo chmod -R 777 /opt/carma/*

echo 'Compiling carma and sourcing. ...'
cd /opt/carma
catkin_make

echo 'Completed, check for compilation errors. Then source and launch manually.'
# Command : cd /opt/carma && source devel/setup.bash && roslaunch carma saxton_cav.launch

