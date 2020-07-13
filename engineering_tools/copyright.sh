#!/bin/bash
echo "This shell script is meant to update the copyright in all CARMA related source files!"

read -sp 'Enter the full folder path: ' DIR
echo
echo 'Folder Path: ' $DIR
read -p 'Is this the correct folder? <y/n> ' prompt1

if [[ $prompt1 == "n" || $prompt1 == "N" || $prompt1 == "no" || $prompt1 == "No" ]]
then
  echo 'You answered' $prompt1', exiting.'
  exit 0
fi

echo "Chosen folder: $DIR"
cd $DIR

read -sp 'Enter the YYYY or YYYY-YYYY to replace: ' OLDYEARTEXT
echo $OLDYEARTEXT
read -sp 'Enter the value to replace it with: ' NEWYEARTEXT
echo $NEWYEARTEXT

OLDCOPYRIGHTTEXT=$"Copyright \(C\) $OLDYEARTEXT LEIDOS"
NEWCOPYRIGHTTEXT=$"Copyright \(C\) $NEWYEARTEXT LEIDOS"

echo "Updating the old copyright: $OLDCOPYRIGHTTEXT"
echo "To the new copyright: $NEWCOPYRIGHTTEXT"

read -p 'Is this the correct, and would you like to proceed to update all the files recursively in the folder? <y/n> ' prompt2

if [[ $prompt2 == "n" || $prompt2 == "N" || $prompt2 == "no" || $prompt2 == "No" ]]
then
  echo 'You answered' $prompt2', exiting.'
  exit 0
fi

echo "Please wait for the update to complete ..."

find . -type f -exec sed -r -i "s/$OLDCOPYRIGHTTEXT/$NEWCOPYRIGHTTEXT/g" {} \;

echo 'The copyright text has been updated in all the files. Please go to github to verify the changes are correct, and manually check it in for peer review. Thank you!'