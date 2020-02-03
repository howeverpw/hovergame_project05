<?php
    // Default time zone
	date_default_timezone_set("America/La_Paz");

	// Send a message to the client
    $nombre_servidor = gethostname();
    $ip_servidor = $_SERVER['SERVER_ADDR'];  //getHostByName(getHostName());
    echo "Received POST data at: " . $nombre_servidor . " " . $ip_servidor . " !\n"; // Just for debugging
	
	//// --------- Store data from received HTTP POST in text file ------------------------------
	if (isset($_POST['unix_time']) && isset($_POST['name']) && isset($_POST['id_code']) &&
      isset($_POST['latitude']) && isset($_POST['longitude']) && isset($_POST['status'])) {
	    // 
    	$unix_time =   $_POST["unix_time"];
    	$name =   $_POST["name"];
    	$id_code =   $_POST["id_code"];
    	$latitude =   $_POST["latitude"];
    	$longitude =   $_POST["longitude"];
    	$status =   $_POST["status"];
    	
    	// Make data string to store in text file
    	$data =  date('D d M Y H:i:s', $unix_time) . ',' . $unix_time . ',' . $name . ',' . $id_code . ',' . $latitude . ','  . $longitude . ',' . $status;
    	
    	// Store data in a text file
        $file =  './data/'. str_replace(' ', '_', $name) . '.txt';
    	// file_put_contents($file, $data, FILE_APPEND | LOCK_EX);
    	file_put_contents($file, $data, LOCK_EX);
	}
	else {
	    // Send a message to the client
	    echo "One or more data pairs weren't received...\n"; // Just for debugging
	}
?>
