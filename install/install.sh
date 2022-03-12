#!/bin/bash

# This is pretty useless, TBH.

rustc -V &> /dev/null
if [ "$?" != "0" ]
then
    echo 'Please install Rust using rustup.'
    exit 1
else
    echo 'Rust found!'
fi

if [[ "$(uname)" == "Linux" ]]
then
    #Someone might run this outside of the working directory.
    cd $(dirname "${BASH_SOURCE[0]}")/..

    read -p "Start a Rust release build? (Y/N)" confirmation

    if [[ "$confirmation" =~ ^y|Y$ ]]
    then
        cargo build --release --bin nano_2022 --features="rplidar"
    fi

    read -p "Install service? Only do this on the actual Jetson nano. (Y/N)" confirmation

    if [[ "$confirmation" =~ ^y|Y$ ]]
    then
        template="$(<install/4904_localization.service)"
        printf "${template//FOLDER_PATH/$(pwd)}" | sudo tee /etc/systemd/system/4904_localization.service >/dev/null
        echo 'Installed using the current folder location. If you move the localization folder, rerun this script.'

        read -p "Enable installed service? (Y/N)" confirmation
        if [[ "$confirmation" =~ ^y|Y$ ]]
        then
            sudo systemctl enable 4904_localization.service
            sudo systemctl start 4904_localization
        fi
    else
        echo 'Service not installed.'
    fi
fi

echo "Done!"
