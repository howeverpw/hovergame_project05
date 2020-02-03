<?php
	echo "----------------------------\r\n";
    $server_name= gethostname();
    $server_ip = $_SERVER['SERVER_ADDR'];  //getHostByName(getHostName());
    echo "Receiving image file at: " . $server_name. " | " . $server_ip . "\r\n";


	$target_dir = "image_data/";
	// $target_dir = $target_dir . basename( $_FILES["uploadFile"]["name"]);
	$target_dir = $target_dir . basename( $_FILES["media"]["name"]);
	$uploadOk=1;

	if (move_uploaded_file($_FILES["media"]["tmp_name"], $target_dir)) {
	    echo "- The file ". basename( $_FILES["media"]["name"]). " has been uploaded.";
	} else {
	    echo "- Sorry, there was an error uploading your file.";
	}