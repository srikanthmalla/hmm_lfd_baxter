function A = steer3d(qr, qn, val, eps)
   qnew = [0 0];
   %If distance > epsilon, then q_new is a point in the direction of q_rand
   if val >= eps
       qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist_3d(qr,qn);
       qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist_3d(qr,qn);
       qnew(3) = qn(3) + ((qr(3)-qn(3))*eps)/dist_3d(qr,qn);
   %else q_new is that q_rand point
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
       qnew(3) = qr(3);
   end
   
   A = [qnew(1), qnew(2), qnew(3)];
end