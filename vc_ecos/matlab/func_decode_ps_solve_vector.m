function [vector,len] = func_decode_ps_solve_vector(filename,format)
    fid = fopen(filename);
    vector_tmp = fscanf(fid,format,inf);
    vector = vector_tmp(1:end-3);
    fclose all;
end