function vector = func_decode_vector(filename,format)
    fid = fopen(filename);
    vector = fscanf(fid,format,inf);
    fclose all;
end