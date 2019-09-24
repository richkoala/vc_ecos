function [vector,len] = func_decode_vector_d(filename)
    fid = fopen(filename);
    vector = fscanf(fid,'%f',inf);
    fclose all;
end