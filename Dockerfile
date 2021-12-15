FROM python
COPY Isse_Copter.py /var
COPY AbstractVirtualCapability.py /var
COPY requirements /var
RUN python -m pip install -r /var/requirements
EXPOSE 9999
CMD python /var/Isse_Copter.py