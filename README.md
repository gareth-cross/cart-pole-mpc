# cart-pole-mpc

<p align="center">
<img src="./media/cart-pole.gif" alt="GIF animation of the cart-pole simulator." width="395">
<br/>
<strong>
<a href="https://garethx.com/posts/cart-pole-mpc/">Blog Post</a> | <a href="https://garethx.com/cart-pole-mpc-app/">Live Demo</a>
</strong>
</p>

![Linux workflow status](https://github.com/gareth-cross/cart-pole-mpc/actions/workflows/linux.yml/badge.svg?branch=main)

`cart-pole-mpc` is a toy implementation of Model Predictive Control (MPC) for a cart-pole system. The controller itself is written in C++, but compiles for the web using [emscripten](https://emscripten.org) and WebAssembly (WASM). I wrote this to learn more about MPC, and also to experiment with deploying to WASM. See the [blog post](https://garethx.com/posts/cart-pole-mpc/) for more context.

## Building

This section outlines the steps required to build the web-app. You will need the following prerequisites:
- CMake >= 3.20
- emscripten: [setup instructions](https://emscripten.org/docs/getting_started/downloads.html)
- Node

### Clone the repo and checkout submodules:
```bash
git clone https://github.com/gareth-cross/cart-pole-mpc
cd cart-pole-mpc
git submodule update --init --recursive
```

### Activate the emscripten SDK:
```bash
source <PATH WHERE YOU CLONED EMSDK>/emsdk_env.sh
```
Make sure that **the build of Node that ships with emscripten has TypeScript installed**. emscripten installs node to `$EMSDK_NODE`. Find the adjacent `npm` executable and run `npm install typescript`.

### Configure the project:
```bash
mkdir build
cd build
emcmake cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_EMSCRIPTEN=ON
```
If you want debug symbols embedded in the WASM, change the build type to `RelWithDebInfo`. C++ function names should then be visible in WASM stacktraces.

### Build the WASM file:
```bash
emmake make -j8
```
**NOTE:** ⚠️ If you had any stray values in `CCFLAGS` or `LDFLAGS` when running the project configuration, these invalid flags may get passed to `em++`. Some package managers like conda will touch these environment variables. I suggest explicitly *clearing* them before configuration.

This step should place a two files in `viz/src`: `optimization-wasm.js` and `optimization-wasm.d.ts`.

### Run the web app:

```bash
cd viz
npm ci
npm run dev
```
🎉 Ta-da! Now you can play with the app locally.

## Changing the dynamics

This project uses [wrenfold](https://github.com/wrenfold/wrenfold) to code-generate the system dynamics. First install wrenfold and SymPy via either `pip` or `conda`.

```bash
pip install wrenfold sympy
```

To change the dynamics, edit `symbolic/dynamics_single.py` and run the generation script:
```bash
python -m symbolic.generate
```

## Contributing

This is not really intended to be a production piece of software, but PRs are welcome if you find something broken. This project uses [pre-commit](https://pre-commit.com) to enforce code-formatting.

The TypeScript/JavaScript components can be formatted by running `npm run prettier-format` in the `viz/dist` directory.

---

cart-pole-mpc is MIT Licensed.
