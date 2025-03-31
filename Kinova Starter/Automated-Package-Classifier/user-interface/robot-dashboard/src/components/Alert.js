import React from "react";

const Alert = ({ children, className }) => {
  return (
    <div className={`p-4 border rounded ${className}`}>
      {children}
    </div>
  );
};

export const AlertTitle = ({ children }) => <h3 className="font-bold">{children}</h3>;
export const AlertDescription = ({ children }) => <p>{children}</p>;

export default Alert;