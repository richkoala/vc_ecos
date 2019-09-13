function [vector,len] = func_decode_vector(filename,format)
    fid = fopen(filename);
    tmp = fscanf(fid,format,inf);
    vector = tmp(1:end-3);
    len    = tmp(end-2);
    fclose all;
end