#!/bin/bash
if [  -z "$1" -o -z "$2" -o -z "$3" ]; then
	echo "$(basename $0) <redhat path> <full version> <package name>" >&2;
	exit 1;
fi

RHPATH="$1";
FULL_VERSION="$2";
PACKAGE_NAME="$3";

tmp=$(mktemp);
NAME="$(git config user.name)";
EMAIL="$(git config user.email)";

echo "* $(LC_ALL=C date "+%a %b %d %Y") $NAME <$EMAIL> [$FULL_VERSION]" >$tmp;
#echo "- redhat: tagging $FULL_VERSION" >>$tmp;
sed -n -e "1,/%changelog/d; /^\-/,/^$/p; /^$/q;" rpm/SPECS/$PACKAGE_NAME.spec >> $tmp;
sed -i -e "/%%CHANGELOG%%/r $tmp" $RHPATH/$PACKAGE_NAME.spec.template;
rm -f "$tmp";

