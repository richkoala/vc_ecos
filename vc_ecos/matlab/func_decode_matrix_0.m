function [mat,mat_nnz_idx] = func_decode_matrix_0(filename,tail_info)
    fid = fopen(filename);
    dat = fscanf(fid,'%d%d%g',inf);
    dat = transpose(reshape(dat,3,[]));
    ROW = dat(end-tail_info,1);
    COL = dat(end-tail_info:2);

    mat = zeros(ROW,COL);
    for idx = 1:length(dat)-tail_info
        mat(dat(idx,1),dat(idx,2))=dat(idx,3);
    end
    
    mat_nnz_idx = dat(1:end-tail_info,:); 
    fclose all;
end