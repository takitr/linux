#!/bin/bash
# This shell script will collect all dtd files for amlogic device tree according to the configuration options.
# You should pass one parameter representing the path of kernel to this shell script.
# meson.dtd file will be created in kernel directory.
# Written by Cai Yun 2013-07-04

#debug
print=echo
#print=test  

TMP_DTD="./arch/arm/boot/meson.dtd"
touch "$TMP_DTD"

copy_fragment(){
    if [ -z "$val_1" ]; then          # no "#ifdef" or "#ifndef", is "#else" ?
        local val_2=`sed -n -e "s/^#else/else/p" "$TMP_FILE"`
        if [ -n "$val_2" ]; then  # key word--"#else"
            noelse=1
            if [ "$nodef" -eq 0 ]; then   # ifdef config
                nodef=1
                if [ "$invalidconfig" -eq 1 ] ; then # ifdef config is not define
                    start_copy=$[$start_copy+1]                             # start copy
                fi
            fi
            if [ "$nodef" -eq 1 ] ; then  # ifndef config
                nodef=0
                if [ "$invalidconfig" -eq 0 ]; then # ifndef config is define
                    start_copy=$[$start_copy+1]                             # start copy
                fi
            fi
        else  # no key word--"#else"
            val_2=`sed -n -e 's/^#endif/END_CONFIG/p' "$TMP_FILE"`
            if [ -z "$val_2" ]; then  # no key word--"#endif"
                if [ "$invalidconfig" -eq "$nodef" ] ; then
                    local val_3=`sed -n -e 's/^sub_file.//p' "$TMP_FILE"`
                    if [ -z "$val_3" ]; then

                        local val_4=`sed -n -e '/^#/p' "$TMP_FILE"`
                        if [ -z "$val_4" ]; then
                            cat "$TMP_FILE" >> "$TMP_DTD"
                        fi
                    fi
                    if [ -n "$val_3" ]; then
                        if [[ IFS != $saveIFS ]] ; then
                            IFS=$saveIFS
                        fi
                        process_file "${path}$val_3"
                    fi
                fi
            fi
            if [ -n "$val_2" ] ; then    # key word--"#endif"
                 if ([ "$invalidconfig" -ne 1 ] || [ "$noelse" -ne 0 ]); then
                    start_copy=$[$start_copy-1]
                 fi
                    invalidconfig=0
                    noelse=0
            fi
        fi
    fi       # no "#ifdef" or "#ifndef"
    
    if [ -n "$val_1" ]; then          # key word--"#ifdef" or "#ifndef", there is a CONFIG
        local CONFIG=`"$path"/scripts/config -s  "$1"`
        if [[ "$CONFIG" = 'y' ]] || [[ "$CONFIG" = 'm' ]] ; then
            invalidconfig=0
            if [ "$nodef" -eq 0 ]; then         # ifdef config is y  or m
                start_copy=$[$start_copy+1]
            fi
        else
            invalidconfig=1
            if [ "$nodef" -eq 1 ]; then         # ifdef config is not define
                start_copy=$[$start_copy+1]
            fi
        fi
    fi
}


process_file(){
    $print "process file $1 start"
    local start_copy=0
    local invalidconfig=0
    local nodef=0
    local noelse=0

    line=`sed -n '$=' $1`

    cat $1 | ( saveIFS="$IFS" ; IFS=$'\\n' ; while read linebuf
    do
        echo "$linebuf" > "$TMP_FILE"
        val_1=`sed -n -e "s/^#ifdef.//p" "$TMP_FILE"`
        if [ -n "$val_1" ]; then      # key word--"#ifdef"
            nodef=0
        else
            val_1=`sed -n -e "s/^#ifndef.//p" "$TMP_FILE"`
            if [ -n "$val_1" ]; then    # key word--"#ifndef"
                nodef=1
            fi  # key word--ifndef
        fi  # key word--ifdef

        copy_fragment "$val_1"
    done)

    if [[ -n $saveIFS ]] ; then      
        IFS=$saveIFS
    fi
    
    $print "process file $1 end"
    $print ""
}   
    
for f in `ls $1/arch/arm/boot/dts/amlogic/aml_top.dtd`; do
        if [ -f $f ] ; then
                $print "$f is exsit"
                path=$1
                TMP_FILE="$1/aml_dtd_tmp"
                touch $TMP_FILE
                chmod 777 $TMP_FILE
                if [ -f $TMP_DTD ] ; then
                    rm $TMP_DTD
                fi
                process_file $f
                rm $TMP_FILE
        fi
done
