Checkout this repository
=
* $ git clone https://projects.cit-ec.uni-bielefeld.de/git/muox_dev

Explanation of folder structure
=
* dokuwiki - The DokuWiki environment !Don't change anything in here, until it is said in the instructions!
* project - All source code resides here. It is also the DokuWiki structure, so all *.txt files build up the content of the DokuWiki 

Install the DokuWiki environment and start with the project
=
* Read: project/dokuwikiInstall.txt
* Follow: project/start.txt


Change rights for proper work
=
* Add the webserver-user to your user group and vice versa
* Make everything writeable for your group: $ sudo chmod -R 770 *
* Make git ignore file mode changes: $ git config core.fileMode false

Use the modules environment
=
* Use NX or SSH to log into the pool room or synthesis computers
* To make the modules permanently in @bash@ available, edit @~/.bashrc@ (e.g. @$ nano ~/.bashrc@) and add the following lines
```
if [ -f /vol/ks-softsync/etc/bashrc ]; then
  source /vol/ks-softsync/etc/bashrc
fi
```
* Optional for @Tcsh@ edit @~/.tcshrc@
```
if( -f /vol/ks-softsync/etc/cshrc ) then
  source /vol/ks-softsync/etc/cshrc
endif
```
* Re-login or source the edited file
* Load the murox environment
```
$ module load murox/env
```
