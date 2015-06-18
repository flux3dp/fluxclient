all: _scanner.so

_scanner.so: src/scanner/scanner.pyx
	cd src; ./setup.py build_ext --inplace
	cp src/_scanner.so fluxghost/scanner/_scanner.so

clean:
	cd src; ./setup.py clean -a
	-@[ -e "src/_scanner.so" ] && rm src/_scanner.so

del: clean
	-@[ -e "fluxghost/scanner/_scanner.so" ] && rm fluxghost/scanner/_scanner.so

testc: _scanner.so