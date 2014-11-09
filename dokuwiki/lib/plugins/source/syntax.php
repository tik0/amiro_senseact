<?php
/**
 * Source Plugin: includes a source file using the geshi highlighter
 *
 * Syntax:     <source filename lang|title>
 *   filename  (required) can be a local path/file name or a remote file uri
 *             to use remote file uri, allow_url_fopen=On must be set in the server's php.ini
 *             filenames with spaces must be surrounded by matched pairs of quotes (" or ')
 *   lang      (optional) programming language name, is passed to geshi for code highlighting
 *             if not provided, the plugin will attempt to derive a value from the file name
 *             (refer $extensions in render() method)
 *   title     (optional) all text after '|' will be rendered above the main code text with a
 *             different style. If no title is present, it will be set to "file: filename"
 *
 *  *** WARNING ***
 *
 *  Unless configured correctly this plugin can be a huge security risk.
 *  Please review/consider
 *    - users who have access to the wiki
 *    - php.ini setting, allow_url_fopen
 *    - php.ini setting, open_basedir
 *    - this plugin's location, allow & deny settings.
 *
 * @license    GPL 2 (http://www.gnu.org/licenses/gpl.html)
 * @author     Christopher Smith <chris@jalakai.co.uk>
 */
if(!defined('DOKU_INC')) die();  // no Dokuwiki, no go

if(!defined('DOKU_PLUGIN')) define('DOKU_PLUGIN',DOKU_INC.'lib/plugins/');
require_once(DOKU_PLUGIN.'syntax.php');

/**
 * All DokuWiki plugins to extend the parser/rendering mechanism
 * need to inherit from this class
 */
class syntax_plugin_source extends DokuWiki_Syntax_Plugin {

  //------------------- [ Security settings ] ---------------------------------------------
  var $location = '';     // prepended to all file names, restricting the filespace exposed to the plugin
  var $allow = array();   // if not empty, ONLY files with the extensions listed will be allowed
  var $deny = array();    // if $allow array is empty, any file with an extension listed in $deny array will be denied
  var $rules = array();   // more complex allow/deny rules, refer documentation

  //------------------------[ Other settings ] ---------------------------------------------
  var $extensions = array(
      'htm' => 'html4strict',
      'html' => 'html4strict',
      'js' => 'javascript'
    );

    /**
     * return some info
     */
    function getInfo(){
      return array(
        'author' => 'Christopher Smith',
        'email'  => 'chris@jalakai.co.uk',
        'date'   => '2008-08-13',
        'name'   => 'Source Plugin',
        'desc'   => 'Include a remote source file
                     Syntax: <source filename #startline-endline language|title>',
        'url'    => 'http://www.dokuwiki.org/plugin:source',
      );
    }

    function getType() { return 'substition'; }
    function getPType() { return 'block'; }
    function getSort() { return 330; }

    /**
     * Connect pattern to lexer
     */
    function connectTo($mode) {
      $this->Lexer->addSpecialPattern('<source.*?>',$mode,substr(get_class($this), 7));
    }

    /**
     * Handle the match
     */
    function handle($match, $state, $pos, &$handler){
      $match = trim(substr($match,7,-1));                    //strip <source from start and > from end

      // ['|"]?<filename>[\1] [#<line#>-<line#>] <lang> | <title>
      list($attr, $title) = preg_split('/\|/u', $match, 2);  //split out title

      $attr = trim($attr);
      $pattern = ($attr{0} == '"' || $attr{0} == "'") ? $attr{0} : '\s';
      list($file, $prop) = preg_split("/$pattern/u", $attr, 3, PREG_SPLIT_NO_EMPTY);

      if (isset($prop) && trim($prop)) {
        $matches = array();
        if (preg_match('/\s*(?:(?:#(\d+)-(\d+))\s*)?(\w+)?/',$prop,$matches)) {
          list(,$start,$end,$lang) = $matches;
          if (!isset($lang)) $lang = '';
        }
      } else {
        $start = $end = $lang = '';
      }

      return array(trim($file), $lang, (isset($title)?trim($title):''), $start, $end);
    }

    /**
     * Create output
     */
    function render($format, &$renderer, $data) {

      $this->_loadSettings();

      list($file, $lang, $title, $start, $end) = $data;
      $ext = substr(strrchr($file, '.'),1);

      $ok = false;
      if (count($this->allow)) {
        if (in_array($ext, $this->allow)) $ok = true;
      } else {
        if (!in_array($ext, $this->deny)) $ok = true;
      }

      // prevent filenames which attempt to move up directory tree by using ".."
      if ($ok && $this->location && preg_match('/(?:^|\/)\.\.(?:\/|$)/', $file)) $ok = false;
      if ($ok && $this->rules) $ok = $this->_checkRules($file);

      if (!$lang) { $lang = isset($this->extensions[$ext]) ? $this->extensions[$ext] : $ext; }

      switch ($format) {
        case 'xhtml' :

          if ($ok && ($source = $this->_getSource($file,$start,$end))) {

            $title = ($title) ? "<span>".hsc($title)."</span>"
                               : $this->_makeTitle($file, $start, $end);

            $renderer->doc .= "<div class='source'><p>$title</p>";
            $renderer->code($source, $lang);
            $renderer->doc .= "</div>";
          } else {
            $error = sprintf($this->getLang('error_file'),hsc($file));
            $renderer->doc .= '<div class="source"><p><span>'.$error.'</span></p></div>';
          }
          break;

        case 'metadata' :
          if ($ok) {
            $renderer->meta['relation']['haspart'][$file] = array('owner'=>$this->getPluginName());
          }
          break;

        default :
          if ($ok) {
            $renderer->code($this->_getSource($file,$start,$end), $lang);
          }
      }

    }

    function _makeTitle($file,$start,$end) {
      $lines = $start ? sprintf($this->getLang('lines'),$start,$end) : '';
      $title = sprintf($this->getLang('title'),hsc($file),$lines);

      return $title;
    }

    function _getSource($file,$start,$end) {

      $source = @file($this->location.$file);
      if (empty($source)) return '';

      // $start is a 1 based index, need to correct to 0 based when slicing arrray
      if (!empty($start)) {
        $lines = count($source);
        if ($start > $lines) {
          $source = $this->getLang('error_start');
        } else if ($end < $start) {
          $source = $this->getLang('error_end');
        } else if ($end > $lines) {
          $source = join('',array_slice($source,$start-1));
        } else {
          $source = join('',array_slice($source,$start-1,$end-$start));
        }
      } else {
        $source = join('',$source);
      }

      return $source;
    }

    function _checkRules($file) {
      $permit = true;
      foreach ($this->rules as $rule) {
        list($allow_deny, $pattern) = $rule;
        if ($allow_deny == 'allow') {
          if (preg_match($pattern,$file)) $permit = true;
        } else {
          if (preg_match($pattern,$file)) $permit = false;
        }
      }

      return $permit;
    }

    function _loadSettings() {
      static $loaded = false;

      if ($loaded) return;

      $this->location = $this->getConf('location');

      $allow = $this->getConf('allow');
      $this->allow = !empty($allow) ? explode('|',$allow) : array();

      $deny = $this->getConf('deny');
      $this->deny = !empty($deny) ? explode('|',$deny) : array();

      $rules = $this->getConf('rules');
      if (!empty($rules)) $this->_parseRules($rules);

      $loaded = true;
    }

    function _parseRules($rules) {
      $rules = explode("\n",$rules);
      foreach ($rules as $rule) {
        $rule = trim($rule);
        if (!$rule || $rule{0} == ';') continue;

        $match = array();
        if (!preg_match('/^(allow|deny)\s+(.+)$/i',$rule,$match)) continue;

        $this->rules[] = array(strtolower($match[1]),$match[2]);
      }
    }
}

//Setup VIM: ex: et ts=4 enc=utf-8 :