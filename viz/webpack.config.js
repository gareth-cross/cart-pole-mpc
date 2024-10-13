const path = require('path');

module.exports = {
    mode: 'development',
    watchOptions: {
        ignored: '**/node_modules',
    },
    entry: './src/index.ts',
    devtool: 'inline-source-map',
    module: {
        rules: [
            {
                test: /\.tsx?$/,
                use: 'ts-loader',
                exclude: /node_modules/,
            },
            {
                test: /\.(scss)$/,
                use: [
                    {
                        loader: 'style-loader'
                    },
                    {
                        loader: 'css-loader'
                    },
                    {
                        loader: 'sass-loader'
                    }
                ]
            },
            {
                test: /\.css$/i,
                include: path.resolve(__dirname, 'src'),
                use: ['style-loader', 'css-loader', 'postcss-loader'],
            },
        ],
    },
    plugins: [],
    resolve: {
        extensions: ['.tsx', '.ts', '.js'],
    },
    output: {
        filename: 'main.js',
        path: path.resolve(__dirname, 'dist'),
    },
    devServer: {
        static: {
            directory: path.resolve(__dirname, 'dist'),
        },
        port: 8000,
        open: false,
        hot: true,
        compress: true,
        historyApiFallback: true,
        client: {
            overlay: {
                errors: true,         // Show critical errors
                warnings: false,      // Suppress warnings
                runtimeErrors: false, // Suppress runtime errors like ResizeObserver errors
            },
        },
    },
};
