<?php
// Based on anexample from:
// https://stackoverflow.com/questions/43681270/how-to-send-a-post-request-on-checkbox-check-uncheck-event
	$command = 0;
    $value = 0;
    if(!empty($_SERVER['HTTP_X_REQUESTED_WITH']) && strtolower($_SERVER['HTTP_X_REQUESTED_WITH']) == 'xmlhttprequest') {
        // var_dump($_POST['id']);
        var_dump($_POST['cmd_value']);
        var_dump($_POST['sel_crew_member_name']);
        var_dump($_POST['sel_crew_member_idx']);
        // $command = $_POST['id'];
        $cmd_value = $_POST['cmd_value'];
        $sel_crew_member_name = $_POST['sel_crew_member_name'];
        $sel_crew_member_idx = $_POST['sel_crew_member_idx'];
    } else {
    	echo "Empty request!" . "\r\n";
    }

    $unix_time =   time(); // Leer la fecha en formato 'tiempo unix' (unix time) GMT del servidor

    // Start XML file, echo parent node
    $xmlString = "<?xml version='1.0' ?>"
				. '<commands>'
				. '<task_mode_cmd '
				
				. 'unix_time="' . $unix_time . '" '
				. 'cmd_value="' . $cmd_value . '" '

				. 'sel_crew_member_name="' . $sel_crew_member_name . '" '
				. 'sel_crew_member_idx="' . $sel_crew_member_idx . '" '

				. '/>'
				. '</commands>';
	$dom = new DOMDocument;
	$dom->preserveWhiteSpace = FALSE;
	$dom->loadXML($xmlString);

	//Save XML as a file
	$dom->save('commands.xml');
?>