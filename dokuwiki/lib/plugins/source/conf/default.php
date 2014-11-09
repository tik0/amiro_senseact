<?php
/*
 * source plugin, default configuration settings
 *
 * @author    Christopher Smith <chris@jalakai.co.uk>
 */

// location is prepended to all file names, restricting the filespace exposed to the plugin
$conf['location'] = '';

// if allow array contains any elements, ONLY files with the extensions listed will be allowed
$conf['allow'] = '';

// if the $allow array is empty, any file with an extension listed in deny array will be denied
$conf['deny'] ='php|asp|pl';

// rules, complex allow/deny rules, refer plugin documentation
$conf['rules'] = '';
