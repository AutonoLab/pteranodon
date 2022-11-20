.PHONY: submodule

submodule: 
	@echo "Updating submodules..."
	git submodule update --init --recursive
	git add . && git commit -m 'Update submodules to latest revisions' && git push
	@echo "   Done."
