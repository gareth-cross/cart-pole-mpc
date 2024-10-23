// Copyright 2024 Gareth Cross.
import mainHtml from './main.html';

// Need to put this in a separate JS file to get TypeScript to shut up about HTML.
export function getUiHtml() {
    return mainHtml;
}
