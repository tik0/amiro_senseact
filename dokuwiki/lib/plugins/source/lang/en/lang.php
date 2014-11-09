<?php
/**
 * english language file
 *
 * @license    GPL 2 (http://www.gnu.org/licenses/gpl.html)
 * @author     Christopher Smith <chris@jalakai.co.uk>
 */

// custom language strings for the plugin
$lang['title'] = 'File: <span>%1$s - %2$s</span>';  //   %1$s = file name, %2$s = line number details
$lang['lines'] = 'lines #%1$u-%2$u';                //   %1$u = starting line number, %2$u = ending line number

$lang['error_file'] = 'Unable to display file &quot;%s&quot;: It may not exist, or permission may be denied.';
$lang['error_start'] = "**ERROR**\nUnable to display any file content.\nThe specified starting line number is more than the number of lines in the file.";
$lang['error_end'] = "**ERROR**\nThe ending line number is less than the starting line number.";

//Setup VIM: ex: et ts=2 enc=utf-8 :