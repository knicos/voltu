export declare type Logger = (message?: any, ...optionalParams: any[]) => void;
export declare function setLogger(log: Logger, error?: Logger): void;
export declare function isEnable(): boolean;
export declare function log(message?: any, ...optionalParams: any[]): void;
export declare function error(message?: any, ...optionalParams: any[]): void;
