# Blockly Example Quick Start

## Create new blocks

- Add new elements to `CUSTOM_BLOCKS` in [jupyterlab-blockly-ipylgbst/src/giskard_blocks_and_toolbox.js](../jupyterlab-blockly-ipylgbst/src/giskard_blocks_and_toolbox.js) or [blocks.js](./blocks.js). (`blocks.js` is just a symbolic link to `giskard_blocks_and_toolbox.js`).
- The extension will automatically rebuild, if not, execute command `jlpm watch` under directory [jupyterlab-blockly-ipylgbst](../jupyterlab-blockly-ipylgbst).
- Refresh the page to reload the extension.

## Python code of the blocks

Python code can be embededd in the Javascript as stings, but it is kind of painful to do so, especially to keep the consistency of the indent.

Here I wrapped the python code into functions under [utils.py](./utils.py). Write python code in the Javascript as less as possible.
