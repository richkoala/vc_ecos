function [vector,len] = func_decode_vector_2(filename,format)
    fid = fopen(filename);
    vector = fscanf(fid,format,inf);
    fclose all;
end