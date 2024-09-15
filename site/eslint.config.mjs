// eslint.config.mjs
// This is copy-pasta from: 
// https://blog.logrocket.com/linting-typescript-eslint-prettier/
import globals from "globals";
import pluginJs from "@eslint/js";
import tseslint from "typescript-eslint";


export default [
  {
    files: ["**/*.{js,mjs,cjs,ts}"]
  },
  {
    files: ["**/*.js"], languageOptions: { sourceType: "commonjs" }
  },
  {
    ignores: ["node_modules/**", "dist/**", "webpack.config.js"]
  },
  { languageOptions: { globals: globals.browser } },
  {
    rules: {
      eqeqeq: "off",
      "no-unused-vars": "error",
      "prefer-const": ["error", { ignoreReadBeforeAssign: true }],
    },
  },
  pluginJs.configs.recommended,
  ...tseslint.configs.recommended,
];
