format:
	find src test -iname *.h -o -iname *.cpp -o -iname *.c | xargs clang-format -i
format-check:
	find src test -iname *.h -o -iname *.cpp -o -iname *.c | xargs clang-format -style=file -output-replacements-xml | grep "<replacement " && exit 1 || exit 0

lint:
	cppcheck --enable=all --suppressions-list=./.suppress.cppcheck --inline-suppr --error-exitcode=1 src test include
