<?php
    // Definir zona horaria para impresion correcta de fechas (si fuera a usarse)
	date_default_timezone_set("America/La_Paz");

	// echo getcwd() . "/data";

	////// --------------------------------------------------------------
	header("Content-type: text/xml");

	// Start XML file, echo parent node
	echo "<?xml version='1.0' ?>";
	echo '<fighters>';

	if ($handle = opendir('data')) {

	    while (false !== ($entry = readdir($handle))) {

	        if ($entry != "." && $entry != "..") {

	            // echo "$entry\n";

	            ////// -------------- Read one file and dump XML ------------
				// $file_contents_string = file_get_contents('Galaxy_Note8.txt');
				$file_contents_string = file_get_contents('data/' . $entry);
				$file_contents_array = explode (",", $file_contents_string);

				// echo $file_contents_string . "..."; // For debugging
				// print_r($file_contents_array);

				$unix_time =   $file_contents_array[1];
				$name =   $file_contents_array[2];
				$id_code =   $file_contents_array[3];
				$latitude =   $file_contents_array[4];
				$longitude =   $file_contents_array[5];
				$status =   $file_contents_array[6];

				// Add to XML document node
				echo '<fighter ';
				echo 'unix_time="' . $unix_time . '" ';
				echo 'name="' . $name . '" ';
				echo 'id_code="' . $id_code . '" ';
				echo 'latitude="' . $latitude . '" ';
				echo 'longitude="' . $longitude . '" ';
				echo 'status="' . $status . '"';
				echo '/>';

	        }
	    }

	    closedir($handle);
	}

	// End XML file
	echo '</fighters>';

	/*
	////// JUST FOR DEBUGGING: Read one file and dump XML ------------
	$file_contents_string = file_get_contents('Galaxy_Note8.txt');
	$file_contents_array = explode (",", $file_contents_string);

	// echo $file_contents_string;
	// print_r($file_contents_array);

	$unix_time =   $file_contents_array[1];
	$name =   $file_contents_array[2];
	$id_code =   $file_contents_array[3];
	$latitude =   $file_contents_array[4];
	$longitude =   $file_contents_array[5];
	$status =   $file_contents_array[6];

	// echo $file_contents_array[1];
	
	// $unix_time =   '123456789';
	// $name =   'Miriam Medrano';
	// $id_code =   'MM123456';
	// $latitude =   '-17.123456';
	// $longitude =   '-66.123456';
	// $status =   'normal';

	header("Content-type: text/xml");

	// Start XML file, echo parent node
	echo "<?xml version='1.0' ?>";
	echo '<fighters>';

	// Add to XML document node
	echo '<fighter ';
	echo 'unix_time="' . $unix_time . '" ';
	echo 'name="' . $name . '" ';
	echo 'id_code="' . $id_code . '" ';
	echo 'latitude="' . $latitude . '" ';
	echo 'longitude="' . $longitude . '" ';
	echo '/>';

	// End XML file
	echo '</fighters>';
	*/
?>