<?php
/**
 * additional setting classes specific to these settings
 *
 * @author    Chris Smith <chris@jalakai.co.uk>
 */

if (!class_exists('setting_source_allowdeny')) {
  class setting_source_allowdeny extends setting_string {

    /**
     *  @return   array(string $label_html, string $input_html)
     */
#    function html(&$plugin, $echo=false) {
#
#    }

    /**
     *  generate string to save setting value to file according to $fmt
     */
#    function out($var, $fmt='php') {
#    }

  }
}

