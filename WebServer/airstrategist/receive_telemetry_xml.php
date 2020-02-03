<?php

	echo "----------------------------\r\n";
    $server_name= gethostname();
    $server_ip = $_SERVER['SERVER_ADDR'];  //getHostByName(getHostName());
    echo "Receiving drone raw telemetry XML at: " . $server_name. " | " . $server_ip . "\r\n";

    //
    $xmlString = trim(file_get_contents('php://input'));
    // echo '$xmlString is:'; print_r($xmlString); echo  "\r\n";

    // echo '$xmlString is of type ' . gettype($xmlString) . "\r\n"; // It must be of type string

	// ---- Save raw telemetry XML data to file
	$fp = fopen('drone_telemetry.xml', 'w');
	$ret_val = fwrite($fp, $xmlString);
	fclose($fp);

	if ($ret_val) {
		echo "[XML Telemetry] Raw telemetry XML saved to file.\r\n";
	}
	else {
		echo "[XML Telemetry] Error while saving raw telemetry XML to file.\r\n";
	}

	$xmlObject=simplexml_load_string($xmlString);


	// ////// ---- JUST FOR DEBUGGING: Load raw telemetry XML data from local file ----
	// echo "----------------------------\r\n";
	// echo "WARN: DEBUG MODE. Reading raw telemetry XML data from local file...\r\n";
	// $xmlObject=simplexml_load_file("drone_telemetry.xml");
	// ////// END:---- JUST FOR DEBUGGING: Load raw telemetry XML data from local file ----

	////// ---- Compute conversion from pixels to meters ----
	// To find ground real distances and areas in meteres
	// Get camera's modified_image_size from telemetry
	$modified_image_width = (double)$xmlObject->modified_image_size->image_width;
	$modified_image_height = (double)$xmlObject->modified_image_size->height;
	// Get drone's relative altitude
	$rel_altitude = (double)$xmlObject->drone_rel_altitude->drone_rel_alt;
	
	$rpicam_fov_widht = 62.2; // From the camera's specs
	$alpha = deg2rad($rpicam_fov_widht / 2.0); // Angle for Pitagorean calculation of conversion coeficient
	$pixels_to_meters = (2*$rel_altitude*tan($alpha)) / $modified_image_width; // Conversion coeficient

	// JUST FOR DEBUGGING: modify coeficiente to see bigger fire spots:
	// $pixels_to_meters = $pixels_to_meters * 10.0;
	$pixels_to_meters = 0.3; //0.15; //1.0;

	// echo '$rel_altitude=' . $rel_altitude . ' | $pixels_to_meters=' . $pixels_to_meters . "\r\n";
	

	////// ---- Convert fire centroid cartesian coordinates to GPS coordinates ----
	// Get drone's GPS coordinates from telemetry
	// $drone_latitude = -17.417266;
	// $drone_longitude = -66.132659;
	$drone_latitude = (double)$xmlObject->drone_gps_coord->drone_latitude;
	$drone_longitude = (double)$xmlObject->drone_gps_coord->drone_longitude;
	// echo '$drone_latitude=' . $drone_latitude . ' | $drone_longitude=' . $drone_longitude;
	$drone_local_altitude = 25.0;

	// Get fire spots centroid coordinate
	$fire_pos_x = (double)$xmlObject->fire_pos_cart_coord->fire_pos_x;
	$fire_pos_y = (double)$xmlObject->fire_pos_cart_coord->fire_pos_y;
	$fire_pos_z = (double)$xmlObject->fire_pos_cart_coord->fire_pos_z;

	// Compute fire centroid cartesian angle
	$fire_pos_angle = atan2($fire_pos_y, $fire_pos_x);

	// Convert cartesian angle to global bearing w.r.t. NORTH
    $distance = sqrt ( pow($fire_pos_x, 2) + pow($fire_pos_y, 2) ) / 1000; // In kilometers
    $distance = $distance * $pixels_to_meters; // Convert from pixels to meters
    $bearing = (450-rad2deg($fire_pos_angle)) % 360; // Convert from angle to bearing

    // Compute fire centroid GPS coordinates w.r.t. drone's GPS coordinates
    // (fire centroid cartesian coordinates are always w.r.t. to drones current position)
    $pointGps = getDestinationPoint($drone_latitude, $drone_longitude, $distance, $bearing);
    // echo 'lat/lon: '. $pointGps[0] . ','. $pointGps[1] . "\r\n";

    // Store computed fire centroid GPS coordinates in XML object
    $xmlObject->fire_pos_cart_coord->fire_pos_x = $pointGps[0]; 
    $xmlObject->fire_pos_cart_coord->fire_pos_y = $pointGps[1];
    ////// END: ---- Convert fire centroid cartesian coordinates to GPS coordinates ----

	////// ---- Convert fire polygons cartesian coordinates to GPS coordinates ----
	// Check if fire polygon 1 is null. It'll be null if no fire spot is detected
	if($xmlObject->fire_polygons_img_coord->poly_1->item != null) {
		// Get the number of points for fire polygon 1 (polygon 1 is the biggest of all received)
		$poly_1_num_elements = $xmlObject->fire_polygons_img_coord->poly_1->item->count();

		// Iterate over all current polygon points to convert their cartesian coord. values
		// to corresponding GPS coordinate values
		for ($idx = 0; $idx < ($poly_1_num_elements-1); $idx = $idx + 2) {
		    // echo (string)$xmlObject->fire_polygons_img_coord->poly_1->item[$idx] . ",";

		    // Convert from local coordinates to GPS coordinates
		    $point_x_coord = (double)$xmlObject->fire_polygons_img_coord->poly_1->item[$idx]; // In pixels
		    $point_y_coord = (double)$xmlObject->fire_polygons_img_coord->poly_1->item[$idx+1]; // In pixels
		    $point_angle = atan2 ( $point_y_coord , $point_x_coord );

		    // echo '$point_x_coord=' . $point_x_coord . ' | $point_y_coord=' . $point_y_coord .' | $point_angle=' . $point_angle . ' | rads='. rad2deg($point_angle) . "\r\n";

		    // Convert cartesian angle to global bearing with respect to NORTH
		    $distance = sqrt ( pow($point_x_coord, 2) + pow($point_y_coord, 2) ) / 1000; // In kilometers
		    $distance = $distance * $pixels_to_meters; // Convert from pixels to meters
		    $bearing = (450-rad2deg($point_angle)) % 360; // Convert from angle to bearing
		    $pointGps = getDestinationPoint($drone_latitude, $drone_longitude, $distance, $bearing);
		    // echo 'lat/lon: '. $pointGps[0] . ','. $pointGps[1] . "\r\n";

		    // Store computed GPS in XML object
		    $xmlObject->fire_polygons_img_coord->poly_1->item[$idx] = $pointGps[0]; 
		    $xmlObject->fire_polygons_img_coord->poly_1->item[$idx+1] = $pointGps[1]; 
		}
	} 
	// // JUST FOR DEBUGGING:
	// else {
	// 	echo 'poly_1 is null'  . "\r\n";
	// }

	// Check if fire polygon 2 is null. It'll be null if no fire spot is detected
	if($xmlObject->fire_polygons_img_coord->poly_2->item != null) {
		// Get the number of points for fire polygon 2 (polygon 1 is the biggest of all received)
		$poly_2_num_elements = $xmlObject->fire_polygons_img_coord->poly_2->item->count();

		// Iterate over all current polygon points to convert their cartesian coord. values
		// to corresponding GPS coordinate values
		for ($idx = 0; $idx < ($poly_2_num_elements-1); $idx = $idx + 2) {
		    // echo (string)$xmlObject->fire_polygons_img_coord->poly_2->item[$idx] . ",";

		    // Convert from local coordinates to GPS coordinates
		    $point_x_coord = (double)$xmlObject->fire_polygons_img_coord->poly_2->item[$idx]; // In pixels
		    $point_y_coord = (double)$xmlObject->fire_polygons_img_coord->poly_2->item[$idx+1]; // In pixels
		    $point_angle = atan2 ( $point_y_coord , $point_x_coord );

		    // echo '$point_x_coord=' . $point_x_coord . ' | $point_y_coord=' . $point_y_coord .' | $point_angle=' . $point_angle . ' | rads='. rad2deg($point_angle) . "\r\n";

		    // Convert cartesian angle to global bearing with respect to NORTH
		    $distance = sqrt ( pow($point_x_coord, 2) + pow($point_y_coord, 2) ) / 1000; // In kilometers
		    $distance = $distance * $pixels_to_meters; // Convert from pixels to meters
		    $bearing = (450-rad2deg($point_angle)) % 360; // Convert from angle to bearing
		    $pointGps = getDestinationPoint($drone_latitude, $drone_longitude, $distance, $bearing);
		    // echo 'lat/lon: '. $pointGps[0] . ','. $pointGps[1] . "\r\n";

		    // Store computed GPS in XML object
		    $xmlObject->fire_polygons_img_coord->poly_2->item[$idx] = $pointGps[0]; 
		    $xmlObject->fire_polygons_img_coord->poly_2->item[$idx+1] = $pointGps[1]; 
		}
	} 
	// // JUST FOR DEBUGGING:
	// else {
	// 	echo 'poly_2 is null'  . "\r\n";
	// }
	
	// Check if fire polygon 3 is null. It'll be null if no fire spot is detected
	if($xmlObject->fire_polygons_img_coord->poly_3->item != null) {
		// Get the number of points for fire polygon 2 (polygon 1 is the biggest of all received)
		$poly_3_num_elements = $xmlObject->fire_polygons_img_coord->poly_3->item->count();
		// echo'$poly_3_num_elements = ' . $poly_3_num_elements  . "\r\n";

		// Iterate over all current polygon points to convert their cartesian coord. values
		// to corresponding GPS coordinate values
		for ($idx = 0; $idx < ($poly_3_num_elements-1); $idx = $idx + 2) {
		    // echo (string)$xmlObject->fire_polygons_img_coord->poly_3->item[$idx] . ",";

		    // Convert from local coordinates to GPS coordinates
		    $point_x_coord = (double)$xmlObject->fire_polygons_img_coord->poly_3->item[$idx]; // In pixels
		    $point_y_coord = (double)$xmlObject->fire_polygons_img_coord->poly_3->item[$idx+1]; // In pixels
		    $point_angle = atan2 ( $point_y_coord , $point_x_coord );

		    // echo '$point_x_coord=' . $point_x_coord . ' | $point_y_coord=' . $point_y_coord .' | $point_angle=' . $point_angle . ' | rads='. rad2deg($point_angle) . "\r\n";

		    // Convert cartesian angle to global bearing with respect to NORTH
		    $distance = sqrt ( pow($point_x_coord, 2) + pow($point_y_coord, 2) ) / 1000; // In kilometers
		    $distance = $distance * $pixels_to_meters; // Convert from pixels to meters
		    $bearing = (450-rad2deg($point_angle)) % 360; // Convert from angle to bearing
		    $pointGps = getDestinationPoint($drone_latitude, $drone_longitude, $distance, $bearing);
		    // echo 'lat/lon: '. $pointGps[0] . ','. $pointGps[1] . "\r\n";

		    // Store computed GPS in XML object
		    $xmlObject->fire_polygons_img_coord->poly_3->item[$idx] = $pointGps[0]; 
		    $xmlObject->fire_polygons_img_coord->poly_3->item[$idx+1] = $pointGps[1]; 
		}
	} 
	// // JUST FOR DEBUGGING:
	// else { 
	// 	echo 'poly_3 is null'  . "\r\n";
	// }

	////// ---- Convert fire polygons areas from pixel^2 to meters^2 ----
	$fire_poly_num_elements = $xmlObject->fire_polygons_area[0]->count();
	// $fire_poly_num_elements = $xmlObject->count();
	// echo '$fire_poly_num_elements=' . $fire_poly_num_elements . "\r\n";


	// Iterate over alla area elements and do the conversion
	for ($idx = 1; $idx <= $fire_poly_num_elements; $idx = $idx + 1) {
		$element = 'n'.$idx; // Elements are tagged n1, n2 and n3

		$fire_poly_area = $xmlObject->fire_polygons_area->$element;

		// echo 'fire_poly area=' . $fire_poly_area . "\r\n";
		// echo 'fire_poly elements=' . $xmlObject->fire_polygons_area->$element . "\r\n";

		// Convert area from pixel^2 to meters^2
		$xmlObject->fire_polygons_area->$element = pow($pixels_to_meters, 2) * $fire_poly_area;
	}


	echo "\r\n";

	// print_r($xmlObject);

	// ---- Save new XML to file
	$xmlConvString = $xmlObject->asXML();
	$fp = fopen('drone_telemetry_conv.xml', 'w');
	$ret_val = fwrite($fp, $xmlConvString);
	fclose($fp);

	if ($ret_val) {
		echo "[XML Telemetry] Converted telemetry XML saved to file.\r\n";
	}
	else {
		echo "[XML Telemetry] Error while saving converted telemetry XML to file.\r\n";
	}

	// $xmlObject->fire_polygons_img_coord->poly_1->item[0] = -17.12345;
	// echo (string)$xmlObject->fire_polygons_img_coord->poly_1->item[0] . "\r\n";

	// $xmlObject->fire_polygons_img_coord->poly_1->item[1] = -66.12345;
	// echo (string)$xmlObject->fire_polygons_img_coord->poly_1->item[1] . "\r\n";

	// print_r($xmlObject->fire_polygons_img_coord->poly_1);
	// echo (string)$xmlObject->fire_polygons_img_coord->poly_1;


	// // Test GPS coord calculatiion from distance and bearing
	// for ($angle = 0; $angle <= 360; $angle = $angle + 90) {

	// 	$bear = (450-$angle) % 360; // Convert from angle to bearing
	// 	echo '$angle =' . $angle . ' | $bear =' . $bear ."\r\n";
	// }
	 
	// $lat = -17.417266;
	// $lon = -66.132659;
	// $distance = 0.050;
	// $bearing = 90;

	// $newGps = getDestinationPoint($lat, $lon, $distance, $bearing);
	// echo $newGps[0] . ','. $newGps[1] . "\r\n";

	// I copied the following function from the Internet:
	// https://stackoverflow.com/questions/7707904/php-calculate-lat-lng-of-the-square-around-a-given-point-on-the-surface
	// Distance is in km, alat and alon are in degrees
	function getDestinationPoint($alat, $alon, $distance, $bearing){
	 $pi=3.14159265358979;
	 $alatRad=$alat*$pi/180;
	 $alonRad=$alon*$pi/180;
	 $bearing=$bearing*$pi/180;
	 $alatRadSin=sin($alatRad);
	 $alatRadCos=cos($alatRad);
	 // Ratio of distance to earth's radius
	 $angularDistance=$distance/6370.997;
	 $angDistSin=sin($angularDistance);
	 $angDistCos=cos($angularDistance);
	 $xlatRad = asin( $alatRadSin*$angDistCos +
	                                   $alatRadCos*$angDistSin*cos($bearing) );
	 $xlonRad = $alonRad + atan2(
	            sin($bearing)*$angDistSin*$alatRadCos,
	            $angDistCos-$alatRadSin*sin($xlatRad));
	 // Return latitude and longitude as two element array in degrees
	 $xlat=$xlatRad*180/$pi;
	 $xlon=$xlonRad*180/$pi;
	 if($xlat>90)$xlat=90;
	 if($xlat<-90)$xlat=-90;
	 while($xlat>180)$xlat-=360;
	 while($xlat<=-180)$xlat+=360;
	 while($xlon>180)$xlon-=360;
	 while($xlon<=-180)$xlon+=360;
	 return array($xlat,$xlon);
	}

?>