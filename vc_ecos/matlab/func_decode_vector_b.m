function [vector,len] = func_decode_vector_b(filename)
    fid = fopen(filename);
    vector_tmp = fscanf(fid,'%f',inf);
    len    = vector_tmp(end-2);
    vector = vector_tmp(1:end-3);
    fclose all;
end